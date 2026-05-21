# app/app_launcher 接口说明

## 模块职责

`app/app_launcher` 是应用层顶层入口。它负责显示启动菜单，并根据按键事件进入 2025 年电赛 E 题基本要求 1、2、3 或设备检查页面。

该模块负责：

- 渲染顶层菜单。
- 用短按切换菜单项。
- 用长按进入当前菜单项。
- 为 E 题基本要求 1 提供 1 到 5 圈的选择入口。

该模块不负责：

- 巡线控制算法。
- 云台视觉跟踪算法。
- 具体外设驱动。
- 题目结果判分。

## 公开接口

### `void App_Launch(void)`

进入应用顶层菜单。该函数通常只由 `main()` 调用，并在内部长期运行。

当前菜单项：

- `E1 Line 1 lap`
- `E1 Line 2 laps`
- `E1 Line 3 laps`
- `E1 Line 4 laps`
- `E1 Line 5 laps`
- `E2 Aim 2s`
- `E3 Aim 4s`
- `Device check`

其中 E1 菜单项分别调用 `AppE_RunLineFollow(1..5)`，用于满足基础要求中 `N=1..5` 圈的选择。
