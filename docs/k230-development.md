# K230 固件与开发连接

本文记录本项目当前使用的 K230 固件、升级注意事项和首选开发连接流程。视觉算法与控制协议见
[`vision-red-line-follow.md`](vision-red-line-follow.md)。

## 当前基线

| 项目 | 当前值 |
|---|---|
| 开发板 | CanMV K230 01Studio - 1G |
| 固件 | CanMV v1.8 01Studio release |
| 完整版本 | `K230-v1.8-0-gc2d1f5cc994c206d0032aaf6aaf09332f3dc3c4c` |
| 板端名称 | `k230_canmv_01studio` |
| VS Code 扩展 | `kendryte747.canmv-vscode` 0.9.6 |
| USB 调试端口 | 当前机器为 `COM15`，实际使用时应重新检测 |
| 板端主程序 | `/sdcard/main.py` |

固件从 [kendryte/canmv_k230 v1.8 release](https://github.com/kendryte/canmv_k230/releases/tag/v1.8)
获取，必须选择与 **01Studio** 板型完全对应的镜像，不可使用其他 K230/K230D 板型镜像。v1.8 更新了 IDE
调试协议、UART/USB、VICAP 流管理、Sensor/CSC 像素格式和显示绑定，因此旧固件上的媒体初始化写法不能
直接视为兼容。

## 更新固件

1. 记录当前板型和固件完整版本，备份 `/sdcard/main.py`、标定参数和其他板端文件。
2. 从官方 v1.8 release 下载 01Studio 对应镜像，并按同页提供的校验文件核对镜像。
3. 使用 01Studio 板卡支持的烧录方式写入镜像。不要保留或混用其他板型的启动文件。
4. 固件更新后把 `/sdcard` 视为可能已被清空；本次升级后原 `main.py` 确实需要重新部署。
5. 用下文的 VS Code CanMV 扩展连接，读取板卡和固件信息，确认板型、版本与上述基线一致。
6. 重新写入 `k230/vision_red_line_follow.py` 为 `/sdcard/main.py`，先由扩展显式运行并检查 Preview，
   再用真正的系统复位验证脱离 IDE 后的开机自启。

固件升级会改变底层媒体行为。升级后至少复测相机模式、双通道视场、LCD 视频层、OSD、Preview、处理帧率
和相机方向，不能只验证脚本能够导入。

## 首选连接方式：VS Code CanMV 扩展

以后日常开发、运行、文件管理、终端输出和相机取帧主要使用 VS Code 的 CanMV 扩展。扩展提供板卡检测、
连接、脚本运行、远程文件、Preview，以及同等能力的 `canmv.mcp` 服务。

### VS Code 界面流程

1. 安装并启用 `kendryte747.canmv-vscode`，打开侧边栏的 CanMV 面板。
2. 执行 `CanMV: 连接开发板`，选择自动检测到的 `CanMV K230`。当前机器通常是 `COM15`，不要写死到
   其他电脑。
3. **连接会使 K230 执行 MicroPython 软复位。** 看到 `MPY: soft reboot` 是正常行为；连接前正在运行的
   `/sdcard/main.py` 会停止，并且本固件不会因此自动恢复项目主循环。
4. 连接完成后，在远程文件树中显式运行 `/sdcard/main.py`，或打开本地脚本后执行
   `CanMV: 在 K230 上运行当前文件`。
5. 先确认终端出现 `vision_v18 sensor=...` 且脚本状态为 running，再执行 `CanMV: 启用预览`。
6. Preview 用于检查完整 800×480 图像、LCD/OSD 合成、相机方向、红线采样点和板端处理帧率。
7. 写入开机程序时使用扩展的“另存为 main.py”或远程文件上传功能；完成后用系统级复位单独验证开机路径。

扩展设置中的 `canmv.baudRate` 默认值为 `12000000`，它是 USB IDE 调试协议参数，不是 K230 与 MSPM0
之间的 TTL UART 波特率。视觉控制串口仍计划使用 `115200 8N1`，当前 `UART_ENABLED = False`。

### MCP/自动化流程

CanMV 扩展 0.9.6 注册了 `canmv.mcp`。自动化连接和抓帧应按以下顺序调用：

```text
canmv_detect_boards
-> canmv_connect_board
-> canmv_execute_file(path="/sdcard/main.py")
-> canmv_script_running
-> canmv_start_preview
-> canmv_get_latest_frame / canmv_save_latest_frame_to_host
-> canmv_stop_preview
-> canmv_disconnect_board
```

关键约束：

- `connect_board` 后必须重新执行 `main.py`，不能假定开机自启程序仍在运行。
- `start_preview` 返回 started 只表示请求已接受；仍须等待新帧并检查 `available=true`。
- 预览帧来自 IDE 帧缓冲。项目的 `Display.init(..., to_ide=True)` 必须保留。
- 连接、Preview 或 MCP 会独占 USB 调试端口。同一时刻不要再用 raw REPL 工具打开相同 COM 端口。
- 当前仓库的 `tools/canmv_mcp_capture.mjs` 可通过扩展后端执行上述流程并保存
  `canmv_preview_latest.jpg`，主要用于本机自动化验证。

## 备用连接方式：raw REPL

`tools/k230_tool.py` 保留为扩展不可用时的备用手段，支持 `list`、`run`、`put`、`cat`、`copy` 和
`reset`。它通过 USB 串口的 MicroPython raw REPL 工作，默认串口速率为 115200，并使用 DTR。

```powershell
python tools/k230_tool.py list
python tools/k230_tool.py --port COM15 put k230/vision_red_line_follow.py /sdcard/main.py
python tools/k230_tool.py --port COM15 cat /sdcard/main.py
```

raw REPL 不能与 VS Code CanMV 扩展同时占用 COM15。若出现“拒绝访问”，先在扩展中停止 Preview 并断开
板卡，不要反复强行打开端口。v1.8 偶尔会在脚本结束时丢失 raw REPL 的结束标记；此时应重新同步并核对
远程文件大小，不能只凭一次超时判断脚本或媒体链路失败。

## VS Code Serial Monitor

本机安装的 Microsoft `ms-vscode.vscode-serial-monitor` 0.13.1 可以直接打开 K230 的 USB CDC 串口并
收发文本。它适合持续观察 `print()` 日志，但不是 CanMV 扩展的替代品。

推荐参数：

| 设置 | 值 |
|---|---|
| Port | 自动检测 K230；当前机器为 `COM15` |
| Baud rate | `115200` |
| Data bits | `8` |
| Parity | `None` |
| Stop bits | `1` |
| Flow control | `None` |
| DTR / RTS | 当前版本默认勾选；本板实测可正常读取 |

实测用上述等价参数打开 COM15 后，Serial Monitor 能持续收到 `vision_v18` 日志，K230 主循环仍保持约
90 FPS，未因打开监视器而软复位。该结论只适用于普通串口监视：

- Serial Monitor 只能显示和发送串口字节，不提供板卡识别协议、远程文件管理、脚本状态或 Preview。
- 当前 `main.py` 不读取 USB 控制台输入，所以在 Monitor 中发送普通文本不会控制视觉程序。
- CanMV 扩展、Serial Monitor、`k230_tool.py` 和其他串口程序不能同时打开同一个 COM 端口。
- 若需要相机画面、运行远程文件或写入 `main.py`，仍应关闭 Serial Monitor 后使用 CanMV 扩展。
- 这里的 115200 是 K230 USB CDC/REPL 监视参数；K230 到 MSPM0 的 TTL UART 也计划使用 115200，
  但二者是不同的物理连接和用途。

## 自启与复位语义

- 上电或 `machine.reset()`：走完整系统启动路径，用于验证 `/sdcard/main.py` 开机自启。
- 扩展连接：会执行 MicroPython 软复位，随后应由扩展显式执行 `/sdcard/main.py`。
- Ctrl-D/raw REPL `reset`：属于软重启调试动作，不能替代最终的开机自启验证。
- Ctrl-C/停止脚本：只中断当前循环；本项目异常退出时不会主动关闭 LCD，以保留最后一帧供排查。

## 已验证结果

在上述固件和扩展组合上已验证：

- 扩展能够识别 `CanMV K230 01Studio - 1G`，协议版本为 2。
- 扩展软复位后可重新执行 `/sdcard/main.py` 并读取终端输出。
- Preview 能获取完整的 800×480 JPEG 帧，视频层无竖纹、无右侧黑屏，方向正确。
- LCD 保持开启时视觉循环约 90 FPS；取帧约 5–8 ms，HSV 掩膜约 4–7 ms，轨迹计算约 0–1 ms。
