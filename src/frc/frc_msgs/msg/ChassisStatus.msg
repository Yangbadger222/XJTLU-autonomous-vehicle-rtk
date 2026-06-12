# 底盘控制模式与手柄按键上行（P3）。
# serial_reader 从 STM32 CSV 第 17/18 字段解析；旧固件下两个字段恒为 0。

std_msgs/Header header

uint8 CTRL_MODE_HOST=0
uint8 CTRL_MODE_GAMEPAD=1
uint8 CTRL_MODE_DISABLED=2

uint8 ctrl_mode
# PS2 按键码（ps2.c PS2_DataKey 编码；0 表示无按键）
uint8 ps2_key
