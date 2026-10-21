# 底盘继电器控制
int8 motor_enable 			  # 电机使能：[0-禁止, 1-使能]，通道1
int8 front_light              # 前照灯：[0-关闭, 1-打开]，通道2
int8 emergency_light          # 报警灯/风扇：[0-关闭, 1-打开]，通道3
int8 tops_auto_belt_off 	  # 上装自动脱带：[0-关闭, 1-打开]，通道4

int8 tops_power_enable        # 上装电源使能，[0-关闭, 1-打开]
int8 injection_valve_enable   # 喷射阀使能
int8 purge_valve_enable       # 清洗阀使能
int8 ac_enable                # 空压机使能
int8 push_actuator            # 推杆控制
int8 horn                     # 蜂鸣器控制
int8 estop                    # 远程急停
int8 mcu_power_enable         # 驱动系统上电
