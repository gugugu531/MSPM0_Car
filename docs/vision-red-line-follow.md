# K230 红线视觉循迹

> **验证状态：尚未整车实测。** 当前只完成 K230 板端识别、LCD/Preview、处理速率和 MSPM0 控制代码
> 验证。相机安装标定、K230→MSPM0 实际 UART、双轮转向符号、控制增益与真实地图循迹均须上车验证；
> 在此之前不得把本文参数视为可直接参赛的最终值。

## 目标与边界

该方案只循参考地图中央的红色轨道，不使用黑色路径或黑色方块。K230 把相机看到的前方红线投影到
地面，并输出轮轴中心处的位置偏差和方向偏差；MSPM0 负责 20 ms 控制、IMU 反馈和双轮差速。
识别对象是车前方纵向延伸的
红色竖线，地图中的红色横支线只视为路口干扰。

K230 程序为 `k230/vision_red_line_follow.py`，目标固件为 CanMV v1.8 01Studio release。
可调阈值与安装标定值集中在文件顶部直接赋值，不经过动态配置层。运行时 LCD 始终开启，
叠加显示红线采样点、拟合结果、置信度、处理帧率和分阶段耗时。

`AUTO_FOCUS_ENABLED = True` 会在 `sensor.run()` 前调用硬件 `auto_focus(True)`，再在 `run()` 后读取
`focus_caps()` 确认模组是否真的有可控焦距范围。支持自动对焦的模组在 LCD 显示 `AF ON`；固定焦镜头
显示 `AF FIXED`；固件接口不可用则显示 `AF N/A`，循迹仍继续运行。自动曝光与自动对焦是两个独立功能，
不能互相替代。

CanMV K230 01Studio 1G、v1.8-0、GC2093 板端实测：`auto_focus(True)` 在 `run()` 前返回 `True`，但
`run()` 后 `focus_caps()` 返回 `(0, 0, 0)`，确认当前原装模组没有 VCM/可控焦距范围，不能靠软件实现
真正自动调焦。v1.8 在 `run()` 前调用 `focus_caps()` 会报 `can't get sensor fd`，所以代码不能照搬文档
示例中先查询能力再启动传感器的顺序。

固件更新、VS Code CanMV 扩展连接、软复位语义和 Preview/MCP 抓帧流程见
[`k230-development.md`](k230-development.md)。后续日常开发以该扩展为首选连接方式，raw REPL 仅作备用。

## 识别链

1. GC2093 工作在 1280×720@90 FPS。通道 0 输出 800×480 YUV420SP 并硬件直绑 LCD；
   通道 2 输出 320×192 RGB888 供识别。两通道共用传感器中央
   `(40, 0, 1200, 720)` 裁剪窗口，视场完全一致，OSD 坐标等比例映射。
2. OpenCV 将 RGB 转为 HSV，并用首尾两个 Hue 区间生成红色掩膜。饱和度下限排除白/灰，
   亮度下限排除黑线和黑色标记块。
3. 对掩膜做 3×3 开运算，去除孤立噪点；所有像素处理均在 OpenCV 原生实现中完成。
4. 在 8 条薄水平 ROI 内调用 `findContours()`，通过面积、宽度和高度筛选竖线片段。
   红色横线产生的宽轮廓直接剔除。
5. LOCK 状态只搜索上一帧拟合轨迹附近，连续丢失后恢复全宽 SEARCH。跨 ROI 连续性选择同一条轨道。
6. 先对图像采样点拟合 `x(y)`，用于像素残差检查、LOCK 预测和 LCD 绘制；再用镜头内参、安装高度和
   俯角把各点的视线与地面求交，在车体地面坐标中拟合 `lateral(forward)`。
7. 地面直线在轮轴位置 `forward=0` 的截距作为横向误差，斜率反正切作为方向误差。这样相机看得较远
   时仍把控制基准换算回轮轴，而不是把前视点误当成车辆当前位置；两项误差经过 EMA 后输出。

## 阈值与安装标定

先架空车轮，直接修改 `vision_red_line_follow.py` 顶部的阈值和标定常量。四个安装量的定义如下，
坐标原点是轮轴中心在地面的投影，车体向右为 `+X`、向前为 `+Y`、向上为 `+Z`：

| 参数 | 单位 | 定义 |
|---|---:|---|
| `K230_PIVOT_AXLE_FORWARD_M` | m | K230 转轴相对轮轴的水平前向距离；转轴在轮轴前方为正 |
| `K230_PIVOT_HEIGHT_M` | m | K230 转轴离地高度 |
| `K230_PIVOT_PITCH_DEG` | ° | 转轴俯角；`0°` 为摄像头水平拍摄，`90°` 为朝向地面 |
| `K230_CAMERA_PIVOT_DISTANCE_M` | m | 转轴沿摄像头光轴到镜头光心的有符号距离，镜头位于光轴前方为正 |

代码据此计算镜头光心相对轮轴的前向位置和离地高度：

```text
camera_forward = pivot_forward + camera_distance * cos(pitch)
camera_height  = pivot_height  - camera_distance * sin(pitch)
```

该距离模型假定转轴到镜头的连线平行于光轴。如果实际支架存在独立的前后或上下偏置，应拆成两个
偏置量后再建模，不能用一个斜距硬凑。文件中的 `0.12 m / 0.24 m / 45° / 0.03 m` 只是可运行初值，
尚未按实车测量。

仅有机械尺寸还不能完成像素到地面的换算。`CAMERA_FOCAL_X_PX`、`CAMERA_FOCAL_Y_PX` 和两个
`CAMERA_PRINCIPAL_*_PX` 是 320×192 检测通道的镜头内参，也必须用当前公共裁剪窗口实测标定；改变
分辨率或裁剪窗口后不能继续沿用。标定顺序为：

1. 在比赛光照下调整 `HSV_RED_LOW_1/HIGH_1` 和 `HSV_RED_LOW_2/HIGH_2`，覆盖红线的两个 Hue 区间。
2. 提高 S 下限直到白底和灰色阴影消失；提高 V 下限直到黑线、黑方块完全不能产生绿色采样点。
3. 测量并写入四个安装参数；标定检测通道的焦距和主点。启动日志会打印计算后的
   `camera_y/camera_h`，先核对它们与实物相符且 `camera_h > 0`。
4. 把车放在直红线上的标准位置和标准朝向，微调 `TRACK_REFERENCE_LATERAL_M` 和
   `TRACK_REFERENCE_HEADING_DEG`，使 LCD/控制台的 `e`、`pos`、`heading` 接近零。
5. 已知距离左右平移车辆，检查 `e` 的米制结果；再把车辆旋转已知小角度，检查 `heading`。右偏和
   车前方轨迹向右倾均定义为正。`POSITION_NORMALIZATION_M` 只决定 UART 中 `position` 的归一化尺度。

默认阈值只是参考地图图片的起点，不能替代真实相机、安装高度、曝光和赛场灯光下的人工标定。

## K230 → MSPM0 协议

当前 K230 程序直接设置 `UART_ENABLED = False`，不会初始化或发送实际控制串口；以下协议代码已保留，
待用户确认接线后再启用。

TTL UART 为 115200 8N1：K230 GPIO3/TX 接 MSPM0 PA9/Debug_Ex RX，并共地。不要把两端 TX 直接相连。

| 偏移 | 字段 | 说明 |
|---:|---|---|
| 0..1 | `A5 5A` | 帧头 |
| 2 | sequence | 每帧加一，8 位回绕 |
| 3 | flags | bit0 红色竖线有效；其余位保留为 0 |
| 4..5 | position | 大端 `int16`，轮轴横向误差除以 `POSITION_NORMALIZATION_M` 后限幅到 `[-1,1]`，再 ×1000 |
| 6..7 | heading | 大端 `int16`，方向偏差 ° ×100 |
| 8 | confidence | 0..255 |
| 9 | checksum | 前 9 字节无符号和的低 8 位 |

MSPM0 在 `Debug_Ex/UART1` ISR 中只搬运字节，`middleware/vision_line_drive` 在任务上下文解析。帧龄超过
120 ms、轨道无效或置信度低于 60 时立即令双轮占空比为零。

## 控制律

状态机只有三个阶段：

```text
WAIT_IMU -> STARTUP_RATE (1 s, gz -> 0) -> TRACK
```

起步阶段复用灰度循迹的角速度稳定方式。进入 `TRACK` 后不再在“循迹”和“角度保持”之间切换：

```text
omega_ref = clamp(Kpos * position_error + Kheading * heading_error, ±100 deg/s)
correction = rate_pid(omega_ref, gyro_z)
left  = base_duty + steering_sign * correction
right = base_duty - steering_sign * correction
```

先在架空轮状态确认 `VISION_LINE_STEERING_SIGN` 和 `VISION_LINE_GYRO_SIGN` 构成负反馈，再从较低基础占空比
整定。位置增益负责回到轨道中心，角度增益负责提前消除切入角；若蛇形明显，优先降低位置增益和视觉
EMA 响应，而不是加入阶段切换。

## K230 测速

`tools/k230/probes/k230_red_line_bench.py` 运行 10 秒有界测试，LCD 保持开启。候选程序应上传到
`/sdcard/vision_red_line_follow_candidate.py`，不会覆盖 `/sdcard/main.py`。最终帧率必须以 LCD 开启、
真实地图进入视野时的 `fps` 为准；无轨道空载结果只能用于排查性能上限。

CanMV v1.8 01Studio release 实测：取帧约 5–7 ms、HSV 双掩膜约 4–5 ms、ROI 跟踪约 0–2 ms，
LCD 实时显示且 OSD 每 5 帧更新时稳态约 90 FPS。该数据仍需用完整实际地图和比赛光照复测识别率。
程序已部署为 `/sdcard/main.py`；v1.8 下 Ctrl-D 只表示 REPL 软重启，最终使用 `machine.reset()`
验证了系统级重启后的自动执行路径。CanMV 扩展连接同样会触发软复位，因此连接后必须由扩展重新执行
`/sdcard/main.py`，再启动 Preview。

## 参考资料

- [CanMV K230 Sensor Module API](https://www.kendryte.com/k230_canmv/en/main/api/mpp/k230_canmv_sensor_module_api_manual.html)：
  `auto_focus()`、`focus_caps()`、双通道尺寸与裁剪接口。
- [CanMV K230 Image Processing API](https://www.kendryte.com/k230_canmv/en/main/api/openmv/image.html)：
  颜色阈值、轮廓与回归接口。
- [OpenMV Linear Regression Line Following](https://openmv.io/blogs/news/linear-regression-line-following)：
  将线的方向误差与位置误差联合用于转向控制。
