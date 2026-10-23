#!/usr/bin/env bash
# 一键查看感知数据：打开 rviz 显示激光雷达点云，再打开三个终端窗口：
#   1) 超声波数据    2) UWB 数据    3) 存储与链路状态（模板画一次，之后只覆盖值，约 1~3 s 刷新一轮）
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
# 渲染方式：启动时画一次固定模板（标题 + 标签 + 分隔线），之后每轮只把新值写回对应行。
#   - 用 ANSI 光标定位 \033[<行>;<列>H 覆盖写值，行尾 \033[K 清掉旧值残尾，不再 clear 整屏，
#     所以屏幕不会整块闪、也不会逐行重绘。
#   - 所有话题并行查询：每个话题独立 timeout 2、同时发起，整轮耗时上限约 2 s；
#     串行查询会让「无数据」话题把整轮拖到「条数 × 2 s」，这才是值一条条往外冒的原因。
#   - 网关状态话题数量不定，每轮扫描一次；数量或名字变化时才整屏重画模板。
status_loop() {
    cd "$CATKIN_DIR" || return 1
    source ./devel/setup.bash

    # —— 布局常量 ——
    # 行号从 1 开始；标签区固定 15 列 + ':' + 空格，值统一从第 18 列开始。
    # 15 列是为了容纳最长的网关名 right_radar_503（正好 15 字符），保证冒号对齐。
    local value_col=18
    local row_title=1
    local row_record=2
    local row_size=3
    local row_dir=4
    local row_lidar=5
    local row_uwb=6
    local row_gw_start=7

    # 固定话题；网关状态话题每轮动态追加
    local -a fixed_topics=(
        /perception_sensors/recording_status
        /perception_sensors/recording_size
        /perception_sensors/recording_dir_size
        /perception_sensors/processed_lidar_connection_status
        /perception_sensors/uwb_status
    )

    # 话题 → data 字段值；空串表示 2 s 内没收到（话题不存在或发布方没起）
    declare -A TOPIC_VALUE

    # 并行取一批话题的 data 字段，结果按话题名存进 TOPIC_VALUE。
    # 每个话题一个后台子 shell + 独立 2 s 超时，整批 wait 完再统一写屏，
    # 因此耗时取决于最慢的一个而不是逐个累加。失败/超时只会让该话题值为空，不影响其它话题。
    fetch_values() {
        local tmpdir topic i=0
        local -a pids=()
        tmpdir=$(mktemp -d) || return 1
        for topic in "$@"; do
            ( timeout 2 rostopic echo -n1 "$topic" 2>/dev/null \
                  | awk '/data:/ {print $NF; exit}' > "$tmpdir/$i" ) &
            pids+=($!)
            i=$((i + 1))
        done
        wait "${pids[@]}" 2>/dev/null || true
        i=0
        for topic in "$@"; do
            TOPIC_VALUE["$topic"]=$(cat "$tmpdir/$i" 2>/dev/null)
            i=$((i + 1))
        done
        rm -rf "$tmpdir"
    }

    # Bool → 中文；空值表示话题还没发布过
    to_bool() {
        case "$1" in
            True)  echo "在线" ;;
            False) echo "离线" ;;
            *)     echo "—（无数据）" ;;
        esac
    }

    to_record() {
        case "$1" in
            True)  echo "录制中" ;;
            False) echo "未录制" ;;
            *)     echo "—（无数据）" ;;
        esac
    }

    # 浮点值原样显示（单位由模板里的标签注明），空值给占位文案
    to_float() {
        echo "${1:-—（无数据）}"
    }

    # 画整屏模板：只在首次进入和网关行数/名字变化时调用。
    # 标签后面不跟值，值由 put_value 按行号写进去。
    render_frame() {
        local gw
        clear
        printf '──────── 存储与链路状态  %s ────────\n' "$(date '+%F %T')"
        printf '录制状态       :\n'
        printf '当前批次大小   :\n'
        printf '保存目录总量   :\n'
        printf '雷达链路       :\n'
        printf 'UWB 链路       :\n'
        for gw in "$@"; do
            printf '%-15s:\n' "$gw"
        done
        printf '──────────────────────────────────────────────────\n'
    }

    # 把值写到指定行的值列，\033[K 清到行尾，新值比旧值短时不会留下旧字符
    put_value() {
        printf '\033[%d;%dH%s\033[K' "$1" "$value_col" "$2"
    }

    # 时间戳每轮刷新，单独覆盖标题行
    put_title() {
        printf '\033[%d;1H──────── 存储与链路状态  %s ────────\033[K' \
            "$row_title" "$(date '+%F %T')"
    }

    local -a gws=() all_topics=()
    local -a last_gws=()
    local frame_drawn=0
    local topic gw v i

    while true; do
        # 一、扫描网关状态话题（名字带网关名，随在线网关增减）
        gws=()
        while IFS= read -r topic; do
            gws+=("${topic##*/}")
        done < <(rostopic list 2>/dev/null | grep '^/perception_sensors/ultrasonic_radar_status/')

        # 二、首次或网关集合变化时重画模板，其余轮次只覆盖值
        if [ "$frame_drawn" -eq 0 ] || [ "${gws[*]}" != "${last_gws[*]}" ]; then
            render_frame "${gws[@]}"
            last_gws=("${gws[@]}")
            frame_drawn=1
        fi

        # 三、固定话题 + 网关话题一次性并行查询
        all_topics=("${fixed_topics[@]}")
        for gw in "${gws[@]}"; do
            all_topics+=("/perception_sensors/ultrasonic_radar_status/$gw")
        done
        fetch_values "${all_topics[@]}"

        # 四、整批结果一次性写回（值都已就绪，不会再一条条冒出来）
        put_title

        v=${TOPIC_VALUE[/perception_sensors/recording_status]}
        put_value "$row_record" "$(to_record "$v")"

        v=${TOPIC_VALUE[/perception_sensors/recording_size]}
        put_value "$row_size" "$(to_float "$v") GB"

        v=${TOPIC_VALUE[/perception_sensors/recording_dir_size]}
        put_value "$row_dir" "$(to_float "$v") GB"

        v=${TOPIC_VALUE[/perception_sensors/processed_lidar_connection_status]}
        put_value "$row_lidar" "$(to_bool "$v")"

        v=${TOPIC_VALUE[/perception_sensors/uwb_status]}
        put_value "$row_uwb" "$(to_bool "$v")"

        i=0
        for gw in "${gws[@]}"; do
            v=${TOPIC_VALUE[/perception_sensors/ultrasonic_radar_status/$gw]}
            put_value $((row_gw_start + i)) "$(to_bool "$v")"
            i=$((i + 1))
        done

        # 光标落到分隔线下方，避免停在值末尾闪烁
        printf '\033[%d;1H' $((row_gw_start + ${#gws[@]} + 1))

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
