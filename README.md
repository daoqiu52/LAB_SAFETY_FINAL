# 实验室安全课堂 FPGA 展示终端

基于安路 EG4S20BG256 的离线音视频教学与安全宣传工程。

## 工程目录

- `LAB_SAFETY_FINAL/src/user_source/`：HDL 源码、IP、引脚及时序约束。
- `LAB_SAFETY_FINAL/src/td_project/lab_ex5_i2s_v1.0.al`：TD 工程入口。
- `LAB_SAFETY_FINAL/TF_CARD/`：TF 卡演示图片和音频素材。

使用 TD 打开 `.al` 工程，按本机安装与目录情况核对路径。综合、布局布线的生成目录和日志未纳入版本管理，可通过 TD 重新生成。

## 功能

图片展示与切换、教学/宣传模式、音频播放与波形显示，以及菜单、字幕和时钟相关功能；具体行为以本仓库 RTL 实现为准。

TF 卡素材包含 `PIC1.BMP`～`PIC4.BMP` 和 `AUDIO.PCM`。PCM 格式为 48 kHz、16 位有符号小端、单声道、无文件头。

修改工程前请阅读 `LAB_SAFETY_FINAL/src/user_source/hdl_source/SD/AGENTS.md`，遵守受保护模块的修改限制。

本仓库保存工程文件，不代表已通过全部时序和上板验证；请以实际构建报告及硬件验证结果为准。
