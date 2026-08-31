#!/bin/bash
# 进入 2UQ2 容器的交互 shell(不启动任何节点)
# 用途: 手动编译 / 跑标定工具 / 调试驱动
#
# 用法:
#   ./run_shell.sh                          # 交互式 shell (ROS 环境已 source)
#   ./run_shell.sh "命令"                   # 直接执行单条命令
#   例: ./run_shell.sh "catkin build ylx2uq2_ros"
#       ./run_shell.sh "cd /root/catkin_ws/src/VINS-Fusion/tools/stereo_extrinsics && g++ -O2 -o stereo_extrinsics stereo_extrinsics.cpp \$(pkg-config --cflags --libs opencv)"
set -e

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# 摄像头设备存在才透传(避免没有摄像头时启动失败)
DEV_FLAG=""
if [ -e /dev/video2uq2 ]; then
    DEV_FLAG="--device /dev/video2uq2"
    echo "  - USB: /dev/video2uq2 已透传"
else
    echo "  - 警告: /dev/video2uq2 不存在, 未透传摄像头(编译/标定不受影响)"
fi

echo "=========================================================="
echo " 2UQ2 容器交互 shell (不启动节点)"
echo "  - 仓库: ${REPO_DIR} -> /root/catkin_ws/src/VINS-Fusion"
echo "  - 提示: 容器内 /root/catkin_ws 每次新建都会重置,"
echo "          首次编译请先: cd /root/catkin_ws && catkin build camera_models ylx2uq2_ros"
echo "=========================================================="

if [ $# -gt 0 ]; then
    exec docker run -it --rm --name vins-shell --net=host \
        -v catkin_ws_vol:/root/catkin_ws \
        ${DEV_FLAG} \
        -v "${REPO_DIR}:/root/catkin_ws/src/VINS-Fusion" \
        ros:vins-fusion bash -lc "$*"
else
    exec docker run -it --rm --name vins-shell --net=host \
        -v catkin_ws_vol:/root/catkin_ws \
        ${DEV_FLAG} \
        -v "${REPO_DIR}:/root/catkin_ws/src/VINS-Fusion" \
        ros:vins-fusion bash
fi
