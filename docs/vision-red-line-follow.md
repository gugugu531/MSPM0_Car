# K230 红线视觉循迹

> **验证状态：尚未整车实测。** 当前只完成 K230 板端识别、LCD/Preview、处理速率和 MSPM0 控制代码
> 验证。相机安装标定、K230→MSPM0 实际 UART、双轮转向符号、控制增益与真实地图循迹均须上车验证；
> 在此之前不得把本文参数视为可直接参赛的最终值。

## 目标与边界

该方案只循参考地图中央的红色轨道，不使用黑色路径或黑色方块。K230 负责输出相机安装坐标系下的
位置偏差和方向偏差；MSPM0 负责 20 ms 控制、IMU 反馈和双轮差速。识别对象是车前方纵向延伸的
红色竖线，地图中的红色横支线只视为路口干扰。

K230 程序为 `k230/vision_red_line_follow.py`，目标固件为 CanMV v1.8 01Studio release。
可调阈值与安装标定值集中在文件顶部直接赋值，不经过动态配置层。运行时 LCD 始终开启，
叠加显示红线采样点、拟合结果、置信度、处理帧率和分阶段耗时。

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
6. 对轨道中心拟合 `x(y)`。近端截距产生归一化位置偏差，斜率产生方向角；两者经过 EMA 后输出。

## 阈值与安装标定

先架空车轮，直接修改 `vision_red_line_follow.py` 顶部的阈值和标定常量：

1. 在比赛光照下调整 `HSV_RED_LOW_1/HIGH_1` 和 `HSV_RED_LOW_2/HIGH_2`，覆盖红线的两个 Hue 区间。
2. 提高 S 下限直到白底和灰色阴影消失；提高 V 下限直到黑线、黑方块完全不能产生绿色采样点。
3. 把车放在直红线上的标准位置和标准朝向，读取 USB 控制台的 `raw_x/raw_heading`，写入
   `TRACK_REFERENCE_X/TRACK_REFERENCE_HEADING_DEG`。
4. 左右移动和小角度旋转车辆，确认 `pos`、`heading` 的正负方向连续且无跳变。

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
| 4..5 | position | 大端 `int16`，归一化位置偏差 ×1000 |
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

`tools/probes/k230_red_line_bench.py` 运行 10 秒有界测试，LCD 保持开启。候选程序应上传到
`/sdcard/vision_red_line_follow_candidate.py`，不会覆盖 `/sdcard/main.py`。最终帧率必须以 LCD 开启、
真实地图进入视野时的 `fps` 为准；无轨道空载结果只能用于排查性能上限。

CanMV v1.8 01Studio release 实测：取帧约 5–7 ms、HSV 双掩膜约 4–5 ms、ROI 跟踪约 0–2 ms，
LCD 实时显示且 OSD 每 5 帧更新时稳态约 90 FPS。该数据仍需用完整实际地图和比赛光照复测识别率。
程序已部署为 `/sdcard/main.py`；v1.8 下 Ctrl-D 只表示 REPL 软重启，最终使用 `machine.reset()`
验证了系统级重启后的自动执行路径。CanMV 扩展连接同样会触发软复位，因此连接后必须由扩展重新执行
`/sdcard/main.py`，再启动 Preview。

## 参考资料

- [CanMV K230 Image Processing API](https://www.kendryte.com/k230_canmv/en/main/api/openmv/image.html)：
  颜色阈值、轮廓与回归接口。
- [OpenMV Linear Regression Line Following](https://openmv.io/blogs/news/linear-regression-line-following)：
  将线的方向误差与位置误差联合用于转向控制。
