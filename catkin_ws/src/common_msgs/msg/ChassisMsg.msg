# 底盘控制
std_msgs/Header header
int8 chassis_emergency                      # 底盘急停：[0-急停禁止, 1-急停使能]
int8 chassis_auto_mode                      # 底盘自动控制模式：[0-未激活, 1-跟随，2-巡迹]
int8 chassis_enable 			            # 底盘使能：[0-禁止, 1-使能]
int8 chassis_auto 			                # 底盘模式：[0-手动模式, 1-自动模式]
int8 chassis_manual_mode                    # 手动模式：[1-近程，2-远程]
int8 chassis_speed_gear			            # 速度挡位：[0-空挡, 1-一挡, 2-二挡, 3-三挡]
float32 chassis_speed_front_back	        # 前后速度：[0.0 ~ 1.0]
float32 chassis_speed_left_right	        # 左右速度：[0.0 ~ 1.0]
common_msgs/ChassisCtrl chassis_relay_ctrl  #底盘继电器控制
common_msgs/TopMsg tops_ctrl                #上装控制
