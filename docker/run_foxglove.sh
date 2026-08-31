#!/bin/bash
# Foxglove 可视化启动脚本 (方案B: 浏览器可视化, 无需 X11)
#
# 用法:
#   ./run_foxglove.sh <VINS配置yaml> [数据目录]        # 数据集模式 (EuRoC bag)
#   ./run_foxglove.sh -d <VINS配置yaml>               # 硬件模式   (2UQ2 驱动 + USB 透传)
#
# 例:
#   ./run_foxglove.sh /root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml ~/datasets
#   ./run_foxglove.sh -d /root/catkin_ws/src/VINS-Fusion/config/2uq2/2uq2_stereo_imu_config.yaml
#
# 起好后:
#   1) 浏览器/桌面版 Foxglove 连接 ws://localhost:9090
#   2) 3D 面板: /vins_estimator/path ; Image: /cam0/image_raw /cam1/image_raw
set -e

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

HW=0
LAUNCH_FILE="${LAUNCH_FILE:-ylx2uq2.launch}"   # 210fps 模式: LAUNCH_FILE=ylx2uq2_210.launch
if [ "$1" == "-d" ] || [ "$1" == "--hw" ]; then
    HW=1
    shift
fi
CONFIG="$1"
DATASET="$2"
if [ -z "$CONFIG" ]; then
    echo "用法: $0 [-d] <VINS配置yaml> [数据目录]"
    echo "  -d/--hw: 硬件模式, 自动启动 2UQ2 驱动"
    exit 1
fi

MOUNT=""
if [ -n "$DATASET" ]; then
    MOUNT="-v ${DATASET}:/root/data"
fi

DEV_FLAG=""
HW_SETUP=""
if [ $HW -eq 1 ]; then
    if [ -e /dev/video2uq2 ]; then
        DEV_FLAG="--device /dev/video2uq2"
        HW_SETUP="catkin build camera_models ylx2uq2_ros && source devel/setup.bash
                  roslaunch ylx2uq2_ros ${LAUNCH_FILE:-ylx2uq2.launch} &"
    else
        echo "警告: /dev/video2uq2 不存在, 不启动驱动。先安装 udev 规则:"
        echo "  sudo cp tools/2uq2_probe/99-2uq2.rules /etc/udev/rules.d/ && sudo udevadm trigger"
    fi
fi

echo "=========================================================="
echo " Foxglove 可视化: $([ $HW -eq 1 ] && echo '[硬件模式] 2UQ2驱动+VINS+rosbridge' || echo '[数据集模式] VINS+rosbridge')"
echo "  - 仓库: ${REPO_DIR} -> /root/catkin_ws/src/VINS-Fusion"
[ -n "$MOUNT" ] && echo "  - 数据集: ${DATASET} -> /root/data"
echo "  - rosbridge: ws://localhost:9090 (Foxglove 连接地址)"
echo "=========================================================="

exec docker run -it --rm --name vins-fusion --net=host \
  -v catkin_ws_vol:/root/catkin_ws \
  ${DEV_FLAG} \
  -v "${REPO_DIR}:/root/catkin_ws/src/VINS-Fusion" \
  ${MOUNT} \
  ros:vins-fusion /bin/bash -c "
    source /root/catkin_ws/devel/setup.bash
    roscore &
    sleep 2
    roslaunch rosbridge_server rosbridge_websocket.launch &
    sleep 2
    ${HW_SETUP}
    sleep 3
    rosrun vins vins_node ${CONFIG}
  "
