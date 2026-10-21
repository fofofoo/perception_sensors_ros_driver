# 单个超声波服务器（一组 4 路探头）的数据。
# 左右侧由 name 区分（left_radar_502 / right_radar_503），
# 字段顺序对应寄存器 1~4 号探头：前、侧1、侧2、后。
string name
float32 Front
float32 Side1
float32 Side2
float32 Back
