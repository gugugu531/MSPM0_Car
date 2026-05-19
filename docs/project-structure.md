# 工程结构

## 顶层结构

当前工程采用“分层源码目录 + IDE 工程目录 + 板级配置目录”的组织方式：

- `app/`：应用入口和任务流程
- `core/`：控制算法和数据处理
- `middleware/`：系统组合能力与共享运行时状态
- `bsp/`：板级外设驱动
- `board/`：SysConfig、启动和链接资源
- `project/`：CCS 和 Keil 工程文件
- `docs/`：工程文档
- `tools/`：调试和烧录脚本

## 运行路径

典型运行路径如下：

1. `app/main.c` 完成系统初始化。
2. `app/app_launcher.c` 显示启动页。
3. 用户选择 `Task flow` 或 `Device check`。
4. 任务流程调用 `core` 中的控制算法。
5. `core` 通过 `middleware` 的组合接口和 `bsp` 的底层驱动控制硬件。

## app

`app` 是顶层应用层，负责：

- 主入口和中断分发
- 启动页
- 菜单树和菜单显示
- 题目模式流程
- 设备自检页面

该层可以直接调用下层公开接口，但不应把纯算法或底层驱动细节继续塞入应用流程。

## core

`core` 放置和硬件无关或弱硬件相关的控制逻辑：

- `pid.*`
- `kinematics.*`
- `rotation.*`
- `tracking.*`
- `sensor_proc.*`
- `step_motor_ctrl.*`

该层可以使用 `middleware` 提供的组合能力，也可以调用必要的 `bsp` 数据接口，但不能包含 `app` 头文件。

## middleware

`middleware` 分为两类：

- `runtime/`：跨模块共享状态声明
- `system/`：系统级组合服务

当前包括错误信息、巡线运行时状态、错误处理、底盘电机组合控制和云台组合控制。

## bsp

`bsp` 是最低层，直接面对板级外设：

- `common/`
- `imu/`
- `key/`
- `laser/`
- `motor/`
- `oled/`
- `step_motor/`
- `time/`
- `tracking_sensor/`

该层禁止包含 `app`、`core`、`middleware` 头文件。需要基础时间或阻塞延时能力时，统一调用 `bsp/time`。

## board

`board` 保存板级配置和启动资源：

- `sys_config/`
- `startup/`

其中 SysConfig 生成文件内容不手改。

## project

`project` 保存 IDE 工程入口：

- `ccs/`
- `keil/`

源码路径重构后必须同步维护这两个目录中的工程文件。
