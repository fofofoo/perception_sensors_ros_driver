#!/usr/bin/env bash
# 一键查看感知数据：打开 rviz 显示激光雷达点云，再打开三个终端窗口：
#   1) 超声波数据    2) UWB 数据    3) 存储与链路状态（每 1~2 s 刷新）
#
# 前提：另一个终端已经运行 ./run.sh 启动了全部节点。
# 退出：Ctrl+C 收起 rviz；三个终端窗口各自独立，直接关窗口即可。
#
# 路径无关：先切到脚本所在仓库的 catkin_ws，再加载 devel/setup.bash。
set -e

cd "$(dirname "$(readlink -f "$0")")/catkin_ws"
source ./devel/setup.bash

# ROS master 没起来时直接提示，避免各终端空等
if ! rostopic list >/dev/null 2>&1; then
    echo "未检测到 ROS master，请先运行 ./run.sh 启动节点" >&2
    exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
    echo "没有图形界面（DISPLAY 为空），无法打开 rviz 与终端窗口" >&2
    exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "未找到 gnome-terminal，无法打开查看终端" >&2
    exit 1
fi

CATKIN_DIR=$(pwd)
export CATKIN_DIR

# —— 一、激光雷达：rviz 显示点云 ——
# 用 lidar_preprocess 自带配置：Fixed Frame = rslidar，显示 10 帧累加后的点云。
# 只想看原始单帧时，把下面配置换成 rslidar_sdk 的 rviz/rviz.rviz（显示 /rslidar_points）。
rviz -d "$(rospack find lidar_preprocess)/rviz/lidar_preprocess.rviz" >/dev/null 2>&1 &
RVIZ_PID=$!
trap 'kill "$RVIZ_PID" 2>/dev/null || true' EXIT

# —— 二、存储与链路状态面板 ——
# 每次重新订阅取一条消息（-n1）：latched 话题立即返回当前值，大小话题在录制中 1 Hz 发布；
# 2 s 拿不到就显示「无数据」，不会把面板卡住。
status_loop() {
    cd "$CATKIN_DIR" || return 1
    source ./devel/setup.bash

    read_value() {
        timeout 2 rostopic echo -n1 "$1" 2>/dev/null | awk '/data:/ {print $NF; exit}'
    }

    # Bool 话题映射成中文，空值表示话题还没发布
    show_bool() {
        case "$(read_value "$1")" in
            True)  echo "在线" ;;
            False) echo "离线" ;;
            *)     echo "—（无数据）" ;;
        esac
    }

    show_record() {
        case "$(read_value /perception_sensors/recording_status)" in
            True)  echo "录制中" ;;
            False) echo "未录制" ;;
            *)     echo "—（无数据）" ;;
        esac
    }

    show_float() {
        local value
        value=$(read_value "$1")
        echo "${value:-—（无数据）}"
    }

    while true; do
        clear
        echo "──────── 存储与链路状态  $(date '+%F %T') ────────"
        printf '录制状态      : %s\n' "$(show_record)"
        printf '当前批次大小  : %s GB\n' "$(show_float /perception_sensors/recording_size)"
        printf '保存目录总量  : %s GB\n' "$(show_float /perception_sensors/recording_dir_size)"
        printf '雷达链路      : %s\n' "$(show_bool /perception_sensors/processed_lidar_connection_status)"
        printf 'UWB 链路      : %s\n' "$(show_bool /perception_sensors/uwb_status)"

        # 超声波每台网关一个状态话题，名字带网关名，从当前话题列表里动态取
        for topic in $(rostopic list | grep '^/perception_sensors/ultrasonic_radar_status/'); do
            printf '%-14s: %s\n' "${topic##*/}" "$(show_bool "$topic")"
        done

        echo "──────────────────────────────────────────────────"
        sleep 1
    done
}

# 把函数定义直接拼进子 shell 命令，避免依赖环境变量跨 gnome-terminal 传递
STATUS_CMD="CATKIN_DIR='$CATKIN_DIR'
$(declare -f status_loop)
status_loop
exec bash"

# —— 三、三个数据终端窗口 ——
# 子 shell 自己 source 一次工作区，保证自定义消息类型可用。
ENV_CMD="cd '$CATKIN_DIR' && source ./devel/setup.bash"

gnome-terminal --window --title="超声波数据" \
    -- bash -c "$ENV_CMD; rostopic echo /perception_sensors/ultrasonic_radar_datas; exec bash" &

gnome-terminal --window --title="UWB 数据" \
    -- bash -c "$ENV_CMD; rostopic echo /perception_sensors/uwb_position; exec bash" &

gnome-terminal --window --title="存储与链路状态" \
    -- bash -c "$STATUS_CMD" &

echo "—— 已打开 rviz + 3 个查看终端，Ctrl+C 收起 rviz ——"

wait
