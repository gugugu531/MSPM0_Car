# K230 调试工具说明

## CanMV IDE K230 安装目录检查

本机 `C:\Program Files\CanMV IDE K230` 主要包含：

- `bin/canmvide.exe`：CanMV IDE 图形界面。
- `lib/qtcreator/plugins/OpenMV.dll`：IDE 的 OpenMV/CanMV 插件。
- `share/qtcreator/examples/03-Machine/uart/`：K230 MicroPython UART 示例。
- `share/qtcreator/html/reference/mpremote.html`：MicroPython `mpremote` 文档。
- `share/qtcreator/dfu-util`、`bossac`、`blhost` 等固件/bootloader 工具。

没有发现可直接用于“上传并运行 K230 Python 脚本”的独立 CLI。当前本机 Python 未安装 `mpremote`，但已安装 `pyserial`。

## 本地工具

`tools/k230_tool.py` 是一个基于 pyserial 的轻量 MicroPython raw REPL 工具，用于在 K230 暴露 USB 串口 REPL 时运行或写入脚本。

列出串口：

```powershell
python tools/k230_tool.py list
```

当前检测结果中：

- `COM15`：更可能是 K230 USB 串口。
- `COM16`：序列号为 `332107134014`，与 DAPLink 调试器一致，应避免用于 K230 脚本。

临时运行脚本：

```powershell
python tools/k230_tool.py --port COM15 run k230/rect_07.py
```

写入为 K230 端 `main.py`：

```powershell
python tools/k230_tool.py --port COM15 put k230/rect_07.py main.py
```

软复位：

```powershell
python tools/k230_tool.py --port COM15 reset
```

## 注意事项

- 使用该工具前应关闭 CanMV IDE 对同一串口的连接，否则串口会被占用。
- 如果 K230 当前运行的脚本占用 REPL 或关闭 USB 串口，`raw REPL` 进入可能失败，需要先在 IDE 中停止运行或重启 K230。
- 该工具不烧录 K230 固件，只通过 MicroPython REPL 运行或写入 `.py` 文件。
