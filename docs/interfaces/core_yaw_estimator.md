# core/yaw_estimator 接口说明

`core/yaw_estimator` 是不读取硬件的纯计算组件，统一维护由融合航向 `B0` 起算的纯角速度
积分航向 `A`。它只负责角度积分和 `B-A` 最短角偏移，不包含 500 ms/1 s 阶段、PID、灰度
判断或电机输出等任务策略。

```c
void YawEstimator_Reset(YAW_ESTIMATOR *estimator);
void YawEstimator_Start(YAW_ESTIMATOR *estimator,
                        float fused_heading_deg,
                        float initial_gyro_z_deg_s);
void YawEstimator_Integrate(YAW_ESTIMATOR *estimator,
                            float gyro_z_deg_s, float dt_s);
float YawEstimator_GetIntegrated(const YAW_ESTIMATOR *estimator);
float YawEstimator_GetInitialFused(const YAW_ESTIMATOR *estimator);
float YawEstimator_GetFusionOffset(const YAW_ESTIMATOR *estimator,
                                   float fused_heading_deg);
```

- `Start(B0, gz0)` 建立 `A0=B0`，并保存区间起点角速度 `gz0`。
- `Integrate(gz1, dt)` 使用梯形公式 `A += (gz0 + gz1) / 2 * dt`，随后保存 `gz1`
  作为下一积分区间起点，并将 A 归一化到 `[-180, 180)`。
- `GetFusionOffset(B)` 返回 `shortest_angle(B-A)`。
- 调用者决定何时冻结偏移以及如何构造参考角；core 不知道“启动”“循迹”等阶段。

当前消费者限定为 `middleware/straight_drive`、`middleware/turn_drive` 和无电机 `Yaw A/B`
检查页，避免继续复制 A/B 算法。
