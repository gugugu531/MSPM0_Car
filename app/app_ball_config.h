/**
 * @file  app_ball_config.h
 * @brief 水管滚球任务的实机标定参数。
 */
#ifndef APP_BALL_CONFIG_H
#define APP_BALL_CONFIG_H

/*
 * pygame_linkage_visualizer.py 的精确几何模型元数据。
 * 固件使用由该模型离线生成的单调查表，不在 Cortex-M0+ 上实时求 asin/atan2。
 * 几何尺寸和曲线形状已确认；水平相位只观测到 200~240 cnt，220 是名义值。
 */
#define APP_BALL_LINKAGE_MODEL_LEVEL_MIN_COUNTS 200.0f
#define APP_BALL_LINKAGE_MODEL_LEVEL_COUNTS 220.0f
#define APP_BALL_LINKAGE_MODEL_LEVEL_MAX_COUNTS 240.0f
#define APP_BALL_LINKAGE_MODEL_TABLE_STEP_COUNTS 20
#define APP_BALL_LINKAGE_MODEL_MAX_INTERP_ERROR_DEG 0.005388f
#define APP_BALL_LINKAGE_MODEL_GEOMETRY_VALIDATED 1U
#define APP_BALL_LINKAGE_MODEL_LEVEL_IS_NOMINAL 1U

/* 已确认：cnt 增大时，水管向使球正向加速的方向倾斜。 */
#define APP_BALL_COUNT_POSITIVE_ACCEL_POSITIVE 1U

#if (APP_BALL_COUNT_POSITIVE_ACCEL_POSITIVE != 1U)
#error "滚球控制律按 cnt 增大=>球正向加速设计，机械方向不符时须显式修改控制符号"
#endif

#endif /* APP_BALL_CONFIG_H */
