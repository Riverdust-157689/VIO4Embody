#!/bin/bash
# Kalibr 安装脚本: 在容器内运行, 安装依赖并编译 kalibr
# 用法: ./kalibr_setup.sh   (容器内: run_shell.sh 进去后执行)
set -e

echo "===== 1/2 安装 Python 依赖 ====="
apt-get update
apt-get install -y python-igraph python-pyx python-scipy python-matplotlib \
                   libv4l-dev python-dev

echo "===== 2/2 编译 kalibr ====="
cd /root/catkin_ws
catkin build kalibr
source devel/setup.bash

echo ""
echo "完成! 检查: which kalibr_calibrate_imucam"
