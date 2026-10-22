#!/usr/bin/env bash
# 一键启动全部感知传感器节点（顶层 launch 依次拉起雷达、超声波、UWB、录制与上装电源联动节点）。
#
# 路径无关：先切到脚本所在仓库的 catkin_ws，从任意目录调用都不会走错。
# systemd 的 ExecStart 直接指向本脚本，因此这里必须保持前台运行。
# 末尾 exec 让 roslaunch 取代当前 shell 成为服务主进程：systemctl stop 发出的 SIGINT
# 会直接到达 roslaunch，由其转发给各节点，bag_recorder 才能让 rosbag 正常落盘。
set -e

cd "$(dirname "$(readlink -f "$0")")/catkin_ws"

# devel/setup.bash 内部会先 source /opt/ros/noetic/setup.bash。
# 未编译（devel 不存在）时在这里直接失败，避免带着不完整的环境启动。
source ./devel/setup.bash

exec roslaunch perception_sensors_client_main perception_sensors_client_main.launch
