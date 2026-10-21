std_msgs/Header header
# 上装控制
int8 top_enable 			    # 上装使能：[0-禁止, 1-使能]

# 消防上装
int8 top_fire_shift             # 消防上装-挡位：[0-0档, 1-1档, 2-2档, 3-3档]
int8 top_fire_mode              # 消防上装-工作模式：[0-清洗, 1-灭火]
int8 top_fire_nozzle            # 消防上装-喷头：[0-关闭, 1-上, 2-下, 3-左, 4-右]
float32 top_fire_driver_b_coe   # 消防上装-驱动器B的系数因子：[0.8 ~ 1.0]

# 水炮喷射控制
int8 top_water_shape            # 水炮-喷射 nozzle：[1-柱状, 2-雾状]
bool top_fire_nozzle_reset      # 水炮-复位 nozzle：[false-否, true-是]

int8 top_ptz_ctrl               # 消防上装-云台控制：[0-关闭, 1-上, 2-下, 3-左, 4-右]

# 轮履摆臂
int8 track_ctrl                 # 履带摆臂：[0-移动控制, 1-摆臂控制-手动, 2-摆臂控制-自动]
int8 track_move                 # 摆臂方向：[0-禁止摆臂, 1-向上摆臂, 2-向下摆臂]
int8 track_zero                 # 摆臂归零：[0-未归零, 1-归零]
int8 track_select               # 摆臂校准选择：[0-无, 1-左前校准, 2-右前校准, 3-左后校准, 4-右后校准]
int8 active_action              # 激活动作：[0-归零, 1-向上, 2-向下, 3-向左, 4-向右]
int8 cluth_front                # 前电磁离合器：[0-断开, 1-连接]
int8 cluth_back                 # 后电磁离合器：[0-断开, 1-连接]
float32 pos_ctrl_front_left     # 前左摆臂位置控制
float32 pos_ctrl_front_right    # 前右摆臂位置控制
float32 pos_ctrl_back_left      # 后左摆臂位置控制
float32 pos_ctrl_back_right     # 后右摆臂位置控制
