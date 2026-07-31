/*
 * sim_ball_scurve.c —— 纯 S 曲线滚球控制的主机端闭环仿真。
 *
 * 直接链接 middleware/ball_scurve 的**固件源码**，所以跑出来的就是真实控制律，
 * 不是另写一份的近似。被建模的非理想因素：
 *
 *   对象   —— 弯曲 α(x)、粗糙度、位置相关静脱离角、滚动阻力、Karnopp 黏滞
 *   执行器 —— 曲柄滑槽几何查表、整数计数量化、步进位置伺服（KP/最低速/限速/10ms 拍）
 *   测量   —— 40 fps 采样、固定时延、高斯噪声、0.1 mm 量化
 *   标定   —— 真实动力学水平点与查表名义 0° 之间的偏差
 *
 * 编译：gcc -O2 -I../../middleware/ball_scurve -o sim_ball_scurve \
 *           sim_ball_scurve.c ../../middleware/ball_scurve/ball_scurve.c -lm
 * 运行：./sim_ball_scurve <scenario> > out.csv
 * 场景：ideal | bow | stick | level | real | real-noff
 */
#include "ball_scurve.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* -std=c11 下 math.h 不导出 M_PI，自带一份免得依赖编译器扩展。 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===== 时间栅格 ===== */
#define SIM_PHYSICS_HZ   2000.0
#define SIM_CONTROL_HZ     50.0    /* app 控制回调周期 20 ms */
#define SIM_STEPPER_HZ    100.0    /* StepMotor_Tick 周期 10 ms */
#define SIM_CAMERA_HZ      40.0

/* ===== 步进与几何（与 step_motor.h / app_ball_task.c 一致）===== */
#define SIM_COUNTS_PER_REV      2000.0
#define SIM_DEG_PER_COUNT       (360.0 / SIM_COUNTS_PER_REV)
#define SIM_SOFT_MIN_COUNTS     20
#define SIM_SOFT_MAX_COUNTS     430
#define SIM_SERVO_KP            3.0
#define SIM_SERVO_MIN_DEG_S     5.0
#define SIM_SERVO_LIMIT_DEG_S   60.0

#define SIM_K_G_MM_S2           7004.75   /* (5/7)·g，mm/s² per rad */
#define SIM_PIPE_HALF_MM        123.5

typedef struct { int count; float angle_deg; } SIM_LINKAGE_POINT;

/* 与 app_ball_task.c 的 H3_LINKAGE_TABLE 逐字一致（180 cnt = 名义 0°）。 */
static const SIM_LINKAGE_POINT SIM_LINKAGE_TABLE[] = {
    {  20, -5.345027f }, {  40, -4.635128f }, {  60, -3.933998f },
    {  80, -3.243158f }, { 100, -2.564173f }, { 120, -1.898660f },
    { 140, -1.248293f }, { 160, -0.614805f }, { 180, +0.000000f },
    { 200, +0.594243f }, { 220, +1.165966f }, { 240, +1.713122f },
    { 260, +2.233576f }, { 280, +2.725092f }, { 300, +3.185334f },
    { 320, +3.611860f }, { 340, +4.002121f }, { 360, +4.353460f },
    { 380, +4.663114f }, { 400, +4.928215f }, { 420, +5.145804f },
    { 430, +5.235833f },
};
#define SIM_LINKAGE_N ((int)(sizeof(SIM_LINKAGE_TABLE)/sizeof(SIM_LINKAGE_TABLE[0])))

static int SimClampCount(int c){
    if (c < SIM_SOFT_MIN_COUNTS){ return SIM_SOFT_MIN_COUNTS; }
    if (c > SIM_SOFT_MAX_COUNTS){ return SIM_SOFT_MAX_COUNTS; }
    return c;
}

static double SimAngleFromCount(double count){
    if (count <= SIM_LINKAGE_TABLE[0].count){ return SIM_LINKAGE_TABLE[0].angle_deg; }
    for (int i = 1; i < SIM_LINKAGE_N; i++){
        if (count <= SIM_LINKAGE_TABLE[i].count){
            const SIM_LINKAGE_POINT *lo = &SIM_LINKAGE_TABLE[i - 1];
            const SIM_LINKAGE_POINT *hi = &SIM_LINKAGE_TABLE[i];
            double f = (count - lo->count) / (double)(hi->count - lo->count);
            return lo->angle_deg + f * (hi->angle_deg - lo->angle_deg);
        }
    }
    return SIM_LINKAGE_TABLE[SIM_LINKAGE_N - 1].angle_deg;
}

static int SimCountFromAngle(double angle_deg){
    if (angle_deg <= SIM_LINKAGE_TABLE[0].angle_deg){ return SIM_LINKAGE_TABLE[0].count; }
    for (int i = 1; i < SIM_LINKAGE_N; i++){
        if (angle_deg <= SIM_LINKAGE_TABLE[i].angle_deg){
            const SIM_LINKAGE_POINT *lo = &SIM_LINKAGE_TABLE[i - 1];
            const SIM_LINKAGE_POINT *hi = &SIM_LINKAGE_TABLE[i];
            double f = (angle_deg - lo->angle_deg) / (hi->angle_deg - lo->angle_deg);
            double c = lo->count + f * (hi->count - lo->count);
            return SimClampCount((int)floor(c + 0.5));
        }
    }
    return SIM_LINKAGE_TABLE[SIM_LINKAGE_N - 1].count;
}

/* ===== 对象参数 ===== */
typedef struct {
    double bow_crown_mm;        /* 管中部相对两端的拱高；正=拱(不稳定)，负=凹(稳定) */
    double roughness_rms_deg;
    double static_threshold_deg;
    double static_variation;
    double rolling_resistance_deg;
    double viscous_per_s;
    double level_error_counts;  /* 真实动力学水平点 − 查表名义 180 cnt */
    double vision_noise_mm;
    double vision_latency_s;
    double vision_bias_mm;
    /**
     * 对象真实 K_G 相对控制器所用名义值的倍率。
     * 几何换算、管身扭转、球是否实心都会让真实增益偏离 (5/7)g，所以控制律
     * **不能**假设这个换算是准的——本项就是用来看它偏了以后还能不能用。
     */
    double gain_scale;
    /** 查表角度相对真实水管角的倍率，模拟连杆几何换算本身的系统性误差。 */
    double linkage_scale;
    /**
     * 曲柄销与滑槽之间的机械回差（间隙），单位：编码器计数。
     *
     * ⚠ **编码器装在电机轴上，看不见回差**——固件以为已经到位，水管其实还没动。
     *   这正是它危险的地方，也是饱和型极限环最经典的成因之一：换向时执行器
     *   有一段死区，等效于在环路里插入一个非线性。
     *
     * 本项此前**完全没有建模**，是仿真复现不出实机 6.4 rad/s 极限环的头号嫌疑。
     * 3D 打印件配合，量级估计 1~4 计数（1 计数 ≈ 0.03° 水管角）。
     */
    double backlash_counts;
    /**
     * 树莓派侧测速的一阶低通系数（每帧）。此前硬编码 0.35。
     * 它在 6.4 rad/s 上贡献的相位不可忽略，而真实滤波形式未知，
     * 所以开成可调项而不是继续假装知道。
     */
    double vision_velocity_alpha;
    /**
     * 速度量化步长，mm/s。树莓派按 int16 mm/s 发送，所以控制器拿到的速度是
     * **整数**。这是环路里的一个量化器，此前未建模。
     */
    double vision_velocity_quantum;
    /**
     * MSPM0 侧的位置外推：控制器实际使用 x = xr + v·age，而不是 xr 本身。
     * 已用实机数据核对（残差 RMS 0.22 mm，即 v/age 的量化）。
     *
     * ⚠ 此前仿真**完全没有这条路径**——它把一个带 62 ms 滞后的速度乘上
     *   测量龄期后加回位置，等于在环路里插入一条额外的反馈通道。
     *   极限环段该项幅值 RMS 1.54 mm，占位置标准差的 28%。
     * 单位 s；0 = 关闭外推。
     */
    double vision_extrapolate_s;
    /** true = 直接把对象真值喂给控制器，用于把控制律本身的误差单独分离出来。 */
    bool ideal_measurement;
    const char *name;
} SIM_PLANT;

/* 与 ball_pipe_model/stepper_balance/model.py 同构的确定性粗糙度。 */
static const double SIM_WAVELENGTH_MM[3] = { 40.0, 18.0, 8.0 };
static const double SIM_WEIGHT[3]        = { 0.8571, 0.4286, 0.2857 };
/*
 * 相位不再写死：它决定了"这一条具体的表面实现"。此前所有仿真结论都建立在
 * 单一种子上，控制参数有可能只是恰好贴合这一条表面。开成可播种后，同一组
 * 参数跑多个种子看的是**分布**而不是单点。
 */
static double sim_phase[3] = { 1.117, 4.402, 2.735 };
static double sim_static_phase = 0.77;

/* 确定性伪随机，保证结果可复现。 */
static unsigned long long sim_rng_state = 20260731ULL;

static double SimNextUniform(void){
    sim_rng_state = sim_rng_state * 6364136223846793005ULL + 1442695040888963407ULL;
    return (double)((sim_rng_state >> 11) & 0xFFFFFFULL) / 16777216.0;
}

/** 用给定种子重置表面相位与噪声流。种子相同 => 结果逐位可复现。 */
static void SimSeedSurface(unsigned long seed){
    sim_rng_state = (unsigned long long)seed * 6364136223846793005ULL + 1442695040888963407ULL;
    for (int i = 0; i < 3; i++){
        sim_phase[i] = 2.0 * M_PI * SimNextUniform();
    }
    sim_static_phase = 2.0 * M_PI * SimNextUniform();
}

static double SimSurfaceSlopeDeg(const SIM_PLANT *p, double x_mm){
    /* 弯曲：h(x) = s(1 − (2x/L)²) 的下坡驱动，slope = 8sx/L²（rad）。 */
    double bow_rad = 8.0 * p->bow_crown_mm * x_mm / (2.0 * SIM_PIPE_HALF_MM) /
                     (2.0 * SIM_PIPE_HALF_MM);
    double bow_deg = bow_rad * 57.29577951308232;
    if (p->roughness_rms_deg <= 0.0){ return bow_deg; }
    double wave = 0.0;
    for (int i = 0; i < 3; i++){
        wave += SIM_WEIGHT[i] * sin(2.0 * M_PI * x_mm / SIM_WAVELENGTH_MM[i] + sim_phase[i]);
    }
    return bow_deg + p->roughness_rms_deg * wave;
}

static double SimStaticThresholdDeg(const SIM_PLANT *p, double x_mm){
    double m = 1.0 + p->static_variation * sin(2.0 * M_PI * x_mm / 40.0 + sim_static_phase);
    if (m < 0.2){ m = 0.2; }
    return p->static_threshold_deg * m;
}

/* ===== 视觉延迟队列 ===== */
#define SIM_DELAY_MAX 4096
typedef struct {
    double buffer[SIM_DELAY_MAX];
    int    write;
    int    delay_samples;
} SIM_DELAY;

static void SimDelayInit(SIM_DELAY *d, double latency_s, double dt, double fill){
    d->delay_samples = (int)floor(latency_s / dt + 0.5);
    if (d->delay_samples >= SIM_DELAY_MAX){ d->delay_samples = SIM_DELAY_MAX - 1; }
    for (int i = 0; i < SIM_DELAY_MAX; i++){ d->buffer[i] = fill; }
    d->write = 0;
}

static double SimDelayStep(SIM_DELAY *d, double value){
    d->buffer[d->write] = value;
    d->write = (d->write + 1) % SIM_DELAY_MAX;
    int read = (d->write - 1 - d->delay_samples + 2 * SIM_DELAY_MAX) % SIM_DELAY_MAX;
    return d->buffer[read];
}

static double SimGauss(void){
    double u1;
    do { u1 = SimNextUniform(); } while (u1 <= 1e-9);
    double u2 = SimNextUniform();
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* ===== 命令行参数 =====
 *
 * 两种调用形式并存，位置形式保留是为了不打断已有脚本（scurve_sim_plot.py）：
 *
 *   位置：sim_ball_scurve <场景> [servo_kp] [限速] [wn] [zeta] [抖幅] [抖频]
 *                          [pd模式] [hold_kd] [move_ki] [积分种子]
 *   命名：sim_ball_scurve --scenario=kick --servo-kp=10 --stick=0.62
 *                          --backlash=2.5 --vel-alpha=0.2 --seed=7 ...
 *
 * 命名参数在**场景预设之后**施加，所以能覆盖预设里的任何一项——这正是
 * 扫鲁棒边界和做对象辨识所需要的。
 */
static const char *ArgStr(int argc, char **argv, const char *key, const char *dflt){
    size_t n = strlen(key);
    for (int i = 1; i < argc; i++){
        if ((strncmp(argv[i], "--", 2) == 0) &&
            (strncmp(argv[i] + 2, key, n) == 0) &&
            (argv[i][2 + n] == '=')){
            return argv[i] + 3 + n;
        }
    }
    return dflt;
}

static double ArgNum(int argc, char **argv, const char *key, double dflt){
    const char *s = ArgStr(argc, argv, key, NULL);
    return (s != NULL) ? atof(s) : dflt;
}

static bool ArgHasNamed(int argc, char **argv){
    for (int i = 1; i < argc; i++){
        if (strncmp(argv[i], "--", 2) == 0){ return true; }
    }
    return false;
}

/** 命名优先、否则回退到位置参数、再否则用默认值。 */
static double PickNum(int argc, char **argv, bool named,
                      const char *key, int pos, double dflt){
    if (named){ return ArgNum(argc, argv, key, dflt); }
    return (argc > pos) ? atof(argv[pos]) : dflt;
}

/* ===== 场景 ===== */
static SIM_PLANT SimMakePlant(const char *scenario){
    SIM_PLANT p;
    memset(&p, 0, sizeof(p));
    p.name = scenario;
    p.vision_latency_s = 0.0;
    p.gain_scale = 1.0;
    p.linkage_scale = 1.0;
    p.backlash_counts = 0.0;          /* 各场景默认无回差，靠 --backlash= 显式打开 */
    p.vision_velocity_alpha = 0.286;   /* 实测：τ=62ms @ 6.4rad/s，两法一致 */
    p.vision_velocity_quantum = 1.0;   /* 树莓派 int16 mm/s */
    p.vision_extrapolate_s = 0.0;      /* 由场景/命令行按测量龄期设定 */
    if (strcmp(scenario, "gain") == 0){
        /* 换算不准：真实滚球增益低 25%，连杆查表角度高 15%。对象其余理想。 */
        p.gain_scale = 0.75;
        p.linkage_scale = 1.15;
        p.ideal_measurement = true;
        return p;
    }
    if (strcmp(scenario, "ideal") == 0){
        /* 只有执行器量化和步进伺服，用来看纯 S 曲线本身的跟踪质量。 */
        p.ideal_measurement = true;
        return p;
    }
    if (strcmp(scenario, "vision") == 0){
        /* 只加测量链（40 fps + 55 ms 时延 + 噪声 + 差分测速），对象保持理想。 */
        p.vision_noise_mm = 0.3;
        p.vision_latency_s = 0.055;
        p.vision_extrapolate_s = 0.046;   /* 实测 age 均值 */
        return p;
    }
    if (strcmp(scenario, "bow") == 0){
        p.bow_crown_mm = 1.5;                 /* 中部拱起 1.5 mm */
        return p;
    }
    if (strcmp(scenario, "stick") == 0){
        p.static_threshold_deg = 0.30;
        p.rolling_resistance_deg = 0.06;
        p.roughness_rms_deg = 0.25;
        p.static_variation = 0.45;
        p.viscous_per_s = 0.10;
        return p;
    }
    if (strcmp(scenario, "level") == 0){
        p.level_error_counts = 15.0;          /* 真实水平点比名义高 15 cnt ≈ 0.45° */
        return p;
    }
    if ((strcmp(scenario, "kick") == 0) || (strcmp(scenario, "ab") == 0)){
        /*
         * 扰动恢复测试：对象按实机辨识结果配置——水平点误差取 +0.254°（≈8.5 cnt，
         * 由 (beam,a) 回归与静置驻留点两法印证），其余非理想因素按实测量级。
         */
        p.bow_crown_mm = 0.8;
        p.roughness_rms_deg = 0.20;
        /*
         * 实测值：2026-07-31 COM16，球停在 −36.9 mm、目标 −50 mm，控制器持续
         * 给出 −0.620° 而球 75 s 一动不动 ⇒ θ_stick ≈ 0.62°。
         * 早期这里填 0.25° 是猜的，会严重低估残差。
         */
        p.static_threshold_deg = 0.62;
        p.static_variation = 0.40;
        p.rolling_resistance_deg = 0.05;
        p.viscous_per_s = 0.10;
        p.level_error_counts = 8.5;
        p.vision_noise_mm = 0.3;
        p.vision_latency_s = 0.045;
        p.vision_extrapolate_s = 0.046;   /* 实测 age 均值 */
        return p;
    }
    /* real / real-noff：全部非理想因素同时打开。 */
    p.bow_crown_mm = 1.5;
    p.roughness_rms_deg = 0.25;
    p.static_threshold_deg = 0.30;
    p.static_variation = 0.45;
    p.rolling_resistance_deg = 0.06;
    p.viscous_per_s = 0.10;
    p.level_error_counts = 15.0;
    p.vision_noise_mm = 0.3;
    p.vision_latency_s = 0.055;
    p.vision_extrapolate_s = 0.046;
    return p;
}

int main(int argc, char **argv){
    bool named = ArgHasNamed(argc, argv);
    const char *scenario = named ? ArgStr(argc, argv, "scenario", "real")
                                 : ((argc > 1) ? argv[1] : "real");
    SIM_PLANT plant = SimMakePlant(scenario);

    /* --- 对象参数覆盖：在场景预设之后施加 --- */
    unsigned long surface_seed = (unsigned long)ArgNum(argc, argv, "seed", 20260731.0);
    SimSeedSurface(surface_seed);
    if (named){
        plant.bow_crown_mm          = ArgNum(argc,argv,"bow",        plant.bow_crown_mm);
        plant.roughness_rms_deg     = ArgNum(argc,argv,"rough",      plant.roughness_rms_deg);
        plant.static_threshold_deg  = ArgNum(argc,argv,"stick",      plant.static_threshold_deg);
        plant.static_variation      = ArgNum(argc,argv,"stick-var",  plant.static_variation);
        plant.rolling_resistance_deg= ArgNum(argc,argv,"roll",       plant.rolling_resistance_deg);
        plant.viscous_per_s         = ArgNum(argc,argv,"viscous",    plant.viscous_per_s);
        plant.level_error_counts    = ArgNum(argc,argv,"level-err",  plant.level_error_counts);
        plant.vision_noise_mm       = ArgNum(argc,argv,"vis-noise",  plant.vision_noise_mm);
        plant.vision_latency_s      = ArgNum(argc,argv,"vis-lag",    plant.vision_latency_s);
        plant.vision_bias_mm        = ArgNum(argc,argv,"vis-bias",   plant.vision_bias_mm);
        plant.gain_scale            = ArgNum(argc,argv,"gain-scale", plant.gain_scale);
        plant.linkage_scale         = ArgNum(argc,argv,"linkage-scale", plant.linkage_scale);
        plant.backlash_counts       = ArgNum(argc,argv,"backlash",   plant.backlash_counts);
        plant.vision_velocity_alpha = ArgNum(argc,argv,"vel-alpha",  plant.vision_velocity_alpha);
        plant.vision_velocity_quantum = ArgNum(argc,argv,"vel-quant", plant.vision_velocity_quantum);
        plant.vision_extrapolate_s  = ArgNum(argc,argv,"extrap",     plant.vision_extrapolate_s);
        plant.ideal_measurement     = (ArgNum(argc,argv,"ideal-meas",
                                       plant.ideal_measurement ? 1.0 : 0.0) != 0.0);
    }
    bool use_feedforward = (strcmp(scenario, "real-noff") != 0);
    /*
     * 步进位置环参数可从命令行覆盖：它们是 step_motor.h 里的 SERVO_KP 与
     * SetSpeedLimit()，而位置环是一阶滞后（时间常数 1/KP），直接决定水管角
     * 能不能跟上剖面要求的角速率。用法：
     *   sim_ball_scurve <scenario> [servo_kp] [speed_limit_deg_s]
     */
    double servo_kp = PickNum(argc,argv,named,"servo-kp",2,SIM_SERVO_KP);
    double servo_limit = PickNum(argc,argv,named,"speed-limit",3,SIM_SERVO_LIMIT_DEG_S);
    /*
     * 外环闭环极点也可从命令行覆盖，用于扫"降带宽能否换到阻尼"：
     *   sim_ball_scurve kick <servo_kp> <speed_limit> <wn> <zeta>
     * kick 场景固定目标 0 mm，在 2/8/14 s 各施加一次速度冲击，测振铃与恢复时间。
     */
    double wn = PickNum(argc,argv,named,"wn",4,2.4);
    double zeta = PickNum(argc,argv,named,"zeta",5,0.9);
    bool kick_mode = (strcmp(scenario, "kick") == 0);
    /* 抖动: sim_ball_scurve <sc> <kp> <lim> <wn> <zeta> <amp_deg> <hz> */
    double dither_amp = PickNum(argc,argv,named,"dither-amp",6,0.0);
    double dither_hz  = PickNum(argc,argv,named,"dither-hz",7,2.0);
    /*
     * argv[8]=1 时退化成**纯 PD**：不生成剖面，参考量直接钉在目标点
     * （x_ref=target, v_ref=0, a_ref=0, ff=0），控制律就只剩 Kp·e + Kd·v。
     * 用同一份代码、同一对象、同一执行器做 A/B，避免"另写一版 PD"带来的
     * 实现差异污染对比。
     * argv[9]=0 时关闭 MOVE/HOLD 调度（hold_kd=0），用于逐项拆解贡献。
     */
    bool pd_mode = (PickNum(argc,argv,named,"pd",8,0.0) != 0.0);
    double hold_kd = PickNum(argc,argv,named,"hold-kd",9,0.020);
    /* 单次移动对照场景：球从 −90 mm 起，目标恒为 0，不走航点序列。 */
    bool ab_mode = (strcmp(scenario, "ab") == 0);
    double move_ki = PickNum(argc,argv,named,"move-ki",10,0.020);
    bool seed_integral = (PickNum(argc,argv,named,"int-seed",11,0.0) != 0.0);
    /*
     * 车加速度扰动剖面：t=car-t0 起加速 car-dur 秒，巡航 car-cruise 秒，
     * 再等时长减速。用于回答"反馈能不能替代前馈"。
     */
    double car_accel     = ArgNum(argc,argv,"car-accel",   0.0);   /* mm/s^2 */
    double car_t0        = ArgNum(argc,argv,"car-t0",      6.0);
    double car_dur       = ArgNum(argc,argv,"car-dur",     1.5);
    double car_cruise    = ArgNum(argc,argv,"car-cruise",  2.0);
    double car_ff_gain   = ArgNum(argc,argv,"car-ff",      0.0);   /* 0=关，1=全额 */

    BALL_SCURVE_CONFIG config = {
        .rolling_acceleration_gain_mm_s2 = SIM_K_G_MM_S2,
        .max_acceleration_mm_s2 = 200.0f,
        .max_velocity_mm_s = 90.0f,
        .min_duration_s = 0.40f,
        .max_duration_s = 3.00f,
        /* Kp = wn²/K_G，Kd = 2ζwn/K_G，rad/m → deg/mm 需 ×57.29578/1000。 */
        .kp_deg_per_mm = (float)(wn * wn / SIM_K_G_MM_S2 * 57.29577951308232),
        .kd_deg_per_mm_s = (float)(2.0 * zeta * wn / SIM_K_G_MM_S2 * 57.29577951308232),
        .feedback_limit_deg = 2.0f,
        .rolling_resistance_deg = 0.0f,   /* 首轮刻意关闭，用于测出实际欠冲量 */
        .rolling_ff_speed_deadband_mm_s = 3.0f,
        .level_bias_deg = 0.0f,           /* 未标定：真实水平误差全额暴露 */
        .angle_min_deg = -5.2f,
        .angle_max_deg = 5.1f,
        .angle_rate_limit_deg_s = 60.0f,
        /*
         * MOVE 段积分，与 app_ball_scurve_task.c 保持一致。
         * argv[10] 可覆盖 Ki（0 = 关闭），argv[11] 可关掉种子，用于逐项 A/B。
         */
        .car_feedforward_gain = (float)car_ff_gain,
        .move_ki_deg_per_mm_s = (float)move_ki,
        .move_integral_limit_deg = 0.6f,
        .move_integral_leak_tau_s = 0.0f,
        .move_integral_apply_in_hold = true,
        .move_integral_min_speed_mm_s = 8.0f,
        .move_integral_seed_from_angle = seed_integral,
        /*
         * MOVE/HOLD 增益调度，与 app_ball_scurve_task.c 保持一致。
         * ⚠ 本仿真**无法验证**它——它针对的 6.4 rad/s 饱和极限环在仿真里
         *   复现不出来（对树莓派侧测速滤波的建模过粗）。放在这里只是保证
         *   仿真跑的是与固件相同的配置，不让两边悄悄分叉。
         */
        .hold_kd_deg_per_mm_s = (float)hold_kd,
        .hold_enter_error_mm = 15.0f,
        .hold_enter_speed_mm_s = 20.0f,
        .hold_enter_dwell_s = 0.3f,
        .hold_exit_error_mm = 30.0f,
        .hold_blend_tau_s = 0.25f,
        .dither_amplitude_deg = (float)dither_amp,
        .dither_frequency_hz = (float)dither_hz,
        .dither_min_error_mm = 3.0f,
        .dither_max_speed_mm_s = 8.0f,
        .dither_dwell_s = 0.5f,
        .settled_position_mm = 3.0f,
        .settled_speed_mm_s = 5.0f,
        .settled_time_s = 0.5f,
        .replan_error_mm = 0.0f,          /* 0 = 纯 S 曲线，不重规划 */
    };
    if (!use_feedforward){
        /* 对照组：砍掉前馈，退化成绕参考点的纯 PD，用来看前馈到底值多少。 */
        config.max_acceleration_mm_s2 = 200.0f;
        config.feedback_limit_deg = 5.0f;
    }

    BALL_SCURVE_CONTROLLER controller;
    BallScurve_Init(&controller);

    /* --- 对象状态 --- */
    double x_mm = 0.0;
    double v_mm_s = 0.0;
    double a_mm_s2 = 0.0;
    bool stuck = true;

    /*
     * --- 执行器状态 ---
     * 电机真实位置是连续量：×32 细分下一个微步只有 0.3125 计数，驱动器按脉冲
     * 频率出力，所以**不能**把每拍位移取整——那会让 5 deg/s 的最低速（0.278
     * 计数/拍）被截断成 0，小误差下电机永远不动。编码器读数才是取整后的值。
     */
    double motor_position_counts = 180.0;
    /*
     * 回差输出：水管实际跟随的位置。与 motor_position_counts 之间隔着机械间隙。
     * ⚠ 编码器读的是 motor_position_counts，**看不见这个间隙**。
     */
    double backlash_out_counts = 180.0;
    int count_actual = 180;
    int count_target = 180;

    /* --- 测量链 --- */
    SIM_DELAY delay;
    double dt = 1.0 / SIM_PHYSICS_HZ;
    /*
     * ⚠ 延迟队列只在相机拍被推进，所以深度必须按**相机周期**折算，不是物理步长。
     * 用物理步长会把 55 ms 变成 110 帧 ≈ 2.75 s，闭环必然发散。
     */
    SimDelayInit(&delay, plant.vision_latency_s, 1.0 / SIM_CAMERA_HZ, 0.0);
    double vision_x = 0.0;
    double vision_v = 0.0;
    double vision_prev_x = 0.0;
    bool vision_have_prev = false;

    /* --- 任务序列：0 → +50 → −50，与要求 3 一致；kick 模式全程守 0 --- */
    double waypoint_mm[3] = { 0.0, 50.0, -50.0 };
    if (kick_mode){ waypoint_mm[1] = 0.0; waypoint_mm[2] = 0.0; }
    if (ab_mode){ waypoint_mm[1] = 0.0; waypoint_mm[2] = 0.0; }
    /* kick 模式的速度冲击时刻与幅值（mm/s），模拟外部拨动小球。 */
    const double kick_time_s[3] = { 3.0, 11.0, 19.0 };
    const double kick_speed[3]  = { 90.0, -120.0, 70.0 };
    int kick_index = 0;
    int waypoint_index = 0;
    double phase_timer = 0.0;
    const double arm_delay_s = 0.5;
    const double dwell_s = 1.8;
    bool planned = false;

    double next_control = arm_delay_s;
    double next_stepper = 0.0;
    double next_camera = 0.0;
    double duration_s = 12.0;

    BALL_SCURVE_OUTPUT out;
    memset(&out, 0, sizeof(out));
    BallScurve_Reset(&controller, (float)SimAngleFromCount(count_actual));

    /*
     * theta_actual = 对象真正感受到的倾角（仿真才知道）；
     * beam         = 固件由编码器查表算出的角，**这才是遥测里能拿到的量**。
     * 二者之差就是水平点标定误差。辨识必须用 beam，否则会假装误差不存在。
     */
    printf("t,phase,target,x,v,a,x_ref,v_ref,a_ref,theta_cmd,theta_actual,beam,"
           "cnt,cnt_tgt,ff,roll_ff,fb,ki,iact,dith,err,stuck,surf,vis_x,vis_v,"
           "active,hold,settled\n");

    if (kick_mode){ duration_s = 26.0; }
    if (ab_mode){ duration_s = 14.0; x_mm = -90.0; }
    if (car_accel != 0.0){ duration_s = car_t0 + 2.0 * car_dur + car_cruise + 6.0; }
    double a_car_now = 0.0;

    for (long step = 0; step * dt <= duration_s; step++){
        double t = step * dt;

        /* 车加速度剖面：加速 → 巡航 → 减速。 */
        if (car_accel != 0.0){
            double u = t - car_t0;
            if (u < 0.0){ a_car_now = 0.0; }
            else if (u < car_dur){ a_car_now = car_accel; }
            else if (u < car_dur + car_cruise){ a_car_now = 0.0; }
            else if (u < 2.0 * car_dur + car_cruise){ a_car_now = -car_accel; }
            else { a_car_now = 0.0; }
        }

        /* ---- 速度冲击：直接改对象速度，模拟外部拨球 ---- */
        if (kick_mode && (kick_index < 3) && (t >= kick_time_s[kick_index])){
            v_mm_s += kick_speed[kick_index];
            stuck = false;
            kick_index++;
        }

        /* ---- 相机：定频采样，经延迟队列、噪声、0.1 mm 量化 ---- */
        if (plant.ideal_measurement){
            vision_x = x_mm;
            vision_v = v_mm_s;
        } else if (t >= next_camera){
            next_camera += 1.0 / SIM_CAMERA_HZ;
            double delayed = SimDelayStep(&delay, x_mm);
            double measured = delayed + plant.vision_bias_mm;
            if (plant.vision_noise_mm > 0.0){
                measured += plant.vision_noise_mm * SimGauss();
            }
            measured = floor(measured * 10.0 + 0.5) / 10.0;
            if (vision_have_prev){
                /*
                 * 树莓派侧的速度就是两帧差分后低通——这正是需要观测器替代的环节，
                 * 本基线刻意保留它，好让它的噪声代价在图上可见。
                 */
                double raw = (measured - vision_prev_x) * SIM_CAMERA_HZ;
                vision_v += plant.vision_velocity_alpha * (raw - vision_v);
            /* 树莓派按 int16 mm/s 发送 —— 环路里的一个量化器。 */
            if (plant.vision_velocity_quantum > 0.0){
                vision_v = plant.vision_velocity_quantum *
                           floor(vision_v / plant.vision_velocity_quantum + 0.5);
            }
            } else{
                vision_have_prev = true;
            }
            vision_prev_x = measured;
            vision_x = measured;
        }

        /* ---- 控制回调 50 Hz ---- */
        if (t >= next_control){
            next_control += 1.0 / SIM_CONTROL_HZ;

            if (!planned){
                BallScurve_PlanTo(&controller, &config, (float)vision_x,
                                  (float)vision_v, (float)waypoint_mm[waypoint_index]);
                if (pd_mode){
                    /*
                     * 纯 PD：把剖面立刻标记为已走完。BallScurve_Update 于是走
                     * "参考量钉在目标点"那条分支，ff/rff 恒为 0，
                     * 控制律精确退化为 Kp·e + Kd·v。
                     */
                    controller.active = false;
                    controller.elapsed_s = controller.duration_s;
                }
                planned = true;
                phase_timer = 0.0;
            }

            /*
             * MSPM0 侧的位置外推：控制器用的是 xr + v·age，不是 xr。
             * v 带 62 ms 滞后，乘上龄期后加回位置 —— 一条额外的反馈通道。
             */
            double control_x = vision_x +
                               vision_v * plant.vision_extrapolate_s;
            BALL_SCURVE_INPUT in = {
                .x_mm = (float)control_x,
                .velocity_mm_s = (float)vision_v,
                .actual_angle_deg = (float)SimAngleFromCount(count_actual),
                .car_acceleration_mm_s2 = (float)a_car_now,
                .dt_s = (float)(1.0 / SIM_CONTROL_HZ),
            };
            BallScurve_Update(&controller, &config, &in, &out);
            count_target = SimCountFromAngle(out.angle_deg);

            /* 剖面走完后驻留 dwell_s，再进入下一个航点。 */
            if (!out.profile_active){
                phase_timer += 1.0 / SIM_CONTROL_HZ;
                if (!kick_mode && !ab_mode && (phase_timer >= dwell_s) && (waypoint_index < 2)){
                    waypoint_index++;
                    planned = false;
                }
            }
        }

        /* ---- 步进位置伺服 100 Hz ---- */
        if (t >= next_stepper){
            next_stepper += 1.0 / SIM_STEPPER_HZ;
            int error = count_target - count_actual;
            if (error != 0){
                double error_deg = abs(error) * SIM_DEG_PER_COUNT;
                double speed = servo_kp * error_deg;
                if (speed < SIM_SERVO_MIN_DEG_S){ speed = SIM_SERVO_MIN_DEG_S; }
                if (speed > servo_limit){ speed = servo_limit; }
                double move = speed / SIM_DEG_PER_COUNT / SIM_STEPPER_HZ;
                double remaining = fabs(count_target - motor_position_counts);
                if (move > remaining){ move = remaining; }
                motor_position_counts += (error > 0) ? move : -move;
                if (motor_position_counts < SIM_SOFT_MIN_COUNTS){
                    motor_position_counts = SIM_SOFT_MIN_COUNTS;
                }
                if (motor_position_counts > SIM_SOFT_MAX_COUNTS){
                    motor_position_counts = SIM_SOFT_MAX_COUNTS;
                }
                count_actual = SimClampCount((int)floor(motor_position_counts + 0.5));
            }
        }

        /* ---- 对象积分 ---- */
        /*
         * 真实动力学水平点与查表名义 0° 差 level_error_counts：把实际计数按该
         * 偏差平移后再查表，得到球实际感受到的倾角。
         */
        /*
         * 机械回差：电机走了不一定水管就跟着走。换向时先要走完间隙，
         * 这段死区是饱和型极限环的经典成因。用**连续**电机位置而非取整后的
         * 编码器读数做输入，因为回差是物理的、与编码器量化无关。
         */
        double play = 0.5 * plant.backlash_counts;
        if (motor_position_counts > backlash_out_counts + play){
            backlash_out_counts = motor_position_counts - play;
        } else if (motor_position_counts < backlash_out_counts - play){
            backlash_out_counts = motor_position_counts + play;
        }
        double theta_actual =
            SimAngleFromCount(backlash_out_counts - plant.level_error_counts) *
            plant.linkage_scale;
        double surface = SimSurfaceSlopeDeg(&plant, x_mm);
        double effective_rad = (theta_actual + surface) * M_PI / 180.0;
        /* 车体系惯性力：ẍ = K_G(sinθ − (a_car/g)cosθ)。 */
        double drive = (sin(effective_rad) - (a_car_now / 9806.65) * cos(effective_rad)) *
                       plant.gain_scale;
        double static_limit = sin(SimStaticThresholdDeg(&plant, x_mm) * M_PI / 180.0);

        if ((fabs(v_mm_s) <= 0.5) && (fabs(drive) <= static_limit)){
            v_mm_s = 0.0;
            a_mm_s2 = 0.0;
            stuck = true;
        } else{
            double dir = (fabs(v_mm_s) > 0.5) ? ((v_mm_s > 0.0) ? 1.0 : -1.0)
                                              : ((drive > 0.0) ? 1.0 : -1.0);
            double resist = sin(plant.rolling_resistance_deg * M_PI / 180.0) * dir;
            a_mm_s2 = SIM_K_G_MM_S2 * (drive - resist) - plant.viscous_per_s * v_mm_s;
            double prev = v_mm_s;
            v_mm_s += a_mm_s2 * dt;
            if ((prev * v_mm_s < 0.0) && (fabs(drive) <= static_limit)){
                v_mm_s = 0.0;
                a_mm_s2 = 0.0;
                stuck = true;
            } else{
                stuck = false;
            }
            x_mm += v_mm_s * dt;
        }
        if (fabs(x_mm) >= SIM_PIPE_HALF_MM){
            x_mm = (x_mm > 0.0) ? SIM_PIPE_HALF_MM : -SIM_PIPE_HALF_MM;
            v_mm_s = 0.0;
            a_mm_s2 = 0.0;
            stuck = true;
        }

        /* ---- 记录：按 100 Hz 输出，够画图也不至于文件过大 ---- */
        if ((step % (long)(SIM_PHYSICS_HZ / 100.0)) == 0){
            printf("%.4f,%d,%.2f,%.4f,%.3f,%.2f,%.4f,%.3f,%.2f,%.4f,%.4f,%.4f,"
                   "%d,%d,%.4f,%.4f,%.4f,%.5f,%d,%.4f,%.4f,%d,%.4f,%.3f,%.3f,"
                   "%d,%d,%d\n",
                   t, waypoint_index, waypoint_mm[waypoint_index],
                   x_mm, v_mm_s, a_mm_s2,
                   (double)out.x_ref_mm, (double)out.v_ref_mm_s, (double)out.a_ref_mm_s2,
                   (double)out.angle_deg, theta_actual,
                   SimAngleFromCount(count_actual),
                   count_actual, count_target,
                   (double)out.feedforward_deg, (double)out.rolling_ff_deg,
                   (double)out.feedback_deg,
                   (double)out.integral_deg, out.integral_active ? 1 : 0,
                   (double)out.dither_deg,
                   waypoint_mm[waypoint_index] - x_mm,
                   stuck ? 1 : 0, surface, vision_x, vision_v,
                   out.profile_active ? 1 : 0, out.hold_mode ? 1 : 0,
                   out.settled ? 1 : 0);
        }
    }

    fprintf(stderr, "scenario=%s  final_x=%.2f mm  target=%.1f mm  error=%.2f mm\n",
            scenario, x_mm, waypoint_mm[2], x_mm - waypoint_mm[2]);
    return 0;
}
