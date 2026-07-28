# tools/probes — 板上调试探测脚本

诊断/探测用途的脚本。日常 K230 开发优先使用 VS Code CanMV 扩展；扩展连接会软复位，连接后须重新
执行 `/sdcard/main.py`。完整流程见 `../../docs/k230-development.md`。以下多数探针仍可经
`../k230_tool.py run <脚本>` 在 K230 上执行，`read_console.py` 为 PC 端直接运行。

| 脚本 | 用途 |
|------|------|
| `read_console.py <COM> [秒]` | PC 端只读监听 K230 串口输出（不打断板上程序） |
| `cvlite_rect_bench.py` | cv_lite 矩形检测有界 bench（帧率/命中统计） |
| `cvlite_live.py` | cv_lite 实时检测遥测 |
| `k230_status_probe.py` | K230 运行时与文件系统状态 |
| `k230_ai_inventory.py` | 列出板上 AI 模型/示例清单 |
| `k230_yolo_probe.py` | YOLOv8n 有界推理探测 |
| `k230_rect_probe.py` | 旧 `find_rects` 候选探测（已被 cv_lite 取代，保留备查） |
| `k230_red_line_bench.py` | v1.8 OpenCV 红色竖线循迹 10 秒有限测试 |
| `k230_v18_capabilities.py` | 查询 v1.8 固件、OpenCV/cv_lite 接口与 Sensor 模式 |
| `k230_v18_cv2_red_probe.py` | HSV 红色掩膜与轮廓分阶段性能探测 |
