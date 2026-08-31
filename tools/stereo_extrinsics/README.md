# stereo_extrinsics 双目外参标定工具

编译(容器内):
```bash
cd /root/catkin_ws/src/VINS-Fusion/tools/stereo_extrinsics
cmake . && make
```

运行:
```bash
./stereo_extrinsics \
  /root/catkin_ws/src/VINS-Fusion/config/2uq2/calib_data/cam0 \
  /root/catkin_ws/src/VINS-Fusion/config/2uq2/calib_data/cam1 \
  /root/catkin_ws/src/VINS-Fusion/config/2uq2/cam0_pinhole.yaml \
  /root/catkin_ws/src/VINS-Fusion/config/2uq2/cam1_pinhole.yaml \
  28.66
```
