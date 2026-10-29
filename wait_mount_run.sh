#!/usr/bin/env bash
# 外挂盘工作区的开机启动引导脚本。
#
# 为什么需要它：感知传感器与跟随控制两套工作区都放在外挂硬盘上（挂载点
# /media/nvidia/sata），而桌面自启动往往比 udisks 自动挂载先执行。自启动项
# 若直接指向盘上的 run.sh / run_follow.sh，开机时文件还不存在，窗口会立刻失败。
# 本脚本先等挂载就绪，再接上入口脚本。
#
# 用法：
#   wait_mount_run.sh run.sh          # 感知传感器
#   wait_mount_run.sh run_follow.sh   # 跟随控制
#
# 部署要求：本脚本必须放在【本地盘】上（如 /home/nvidia/scripts/），再由
# ~/.config/autostart/*.desktop 调用。若它本身放在外挂盘，硬盘没挂载时脚本
# 文件都不存在，等待逻辑根本没有机会执行。
#
# 行为：挂载点就绪且入口脚本存在后 exec 接管终端；最多等 WAIT_SEC 秒，超时
# 打印原因并停住等回车，避免 gnome-terminal 在子进程退出后立刻关窗、错误一闪而过。
set -u

MOUNT_DIR=/media/nvidia/sata                                # 外挂盘挂载点（udisks 自动挂载路径）
WORKSPACE="$MOUNT_DIR/code/perception_sensors_ros_driver"   # 工作区在盘上的位置
ENTRY=${1:-run.sh}                                          # 盘上工作区根目录的入口脚本
WAIT_SEC=300                                                # 最长等待秒数；大盘 fsck / 识别慢时留足余量

for ((i = 0; i < WAIT_SEC; i++)); do
    # mountpoint -q 只认真正的挂载：挂载点目录可能先被创建、盘却还没挂上，
    # 单看目录或文件存在会误判；再叠加入口文件检查，确保工作区内容已可见
    if mountpoint -q "$MOUNT_DIR" && [ -f "$WORKSPACE/$ENTRY" ]; then
        cd "$WORKSPACE" || exit 1
        # exec 让入口脚本取代当前 shell：终端前台进程链保持连续，
        # Ctrl+C 的 SIGINT 能直达 roslaunch，bag_recorder 才能正常落盘
        exec bash "$ENTRY"
    fi
    sleep 1
done

echo "[错误] ${WAIT_SEC}s 内未等到 $MOUNT_DIR 挂载或 $WORKSPACE/$ENTRY 不存在，本次未启动" >&2
read -r -p "按回车关闭窗口" _
exit 1
