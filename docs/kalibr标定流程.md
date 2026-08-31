# Kalibr IMU-相机外参标定全流程（2UQ2 实机验证版）

> 目的：标定 IMU↔相机外参（T_imu_cam）、时间偏移、IMU 噪声参数。
> 产出：`kalibr_config/results/` 下的外参 yaml、imu yaml、报告 pdf。
> 本流程已在本机完整跑通（重投影误差 0.21/0.23px，收敛良好）。

## 0. kalibr/ 文件夹的来源

`kalibr/` 是克隆的官方源码仓库（ethz-asl/kalibr），放在**仓库根目录**以便挂载进容器编译：
```bash
git clone --depth 1 https://github.com/ethz-asl/kalibr.git kalibr
```
- 已加入 `.dockerignore`（重建镜像时不会被打包/编译，不影响 `make build`）
- 容器运行时通过挂载区访问源码（Python 脚本直接跑，C++ 部分需编译一次）

## 1. 安装（容器内，一次）

```bash
# run_shell.sh 进容器后:
bash /root/catkin_ws/src/VINS-Fusion/tools/kalibr_setup.sh
# = 自动执行:
#   apt-get install -y python-igraph python-pyx python-scipy python-matplotlib libv4l-dev python-dev
#   cd /root/catkin_ws && catkin build kalibr
```
> 依赖坑：`ethz_apriltag2` 编译报 `libv4l2.h: No such file` → 装 `libv4l-dev`。
> 容器是 `--rm`，但 `/root/catkin_ws` 已挂持久卷（`catkin_ws_vol`），**编译产物保留**；apt 依赖每次要重装（脚本一条命令）。

## 2. 打补丁（3 个必须，都在源码里）

| 坑 | 现象 | 补丁 |
|---|---|---|
| 多进程提取死锁 | 角点提取**永远卡在 Progress 19**（换数据也一样） | `kalibr/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py` 第64行：`multithreading = False`（容器里 fork+OpenCV 死锁，强制单线程；注释必须用**英文**，Python2 报编码错） |
| 无显示器崩溃 | `QXcbConnection: Could not connect to display` | 运行时 `export MPLBACKEND=Agg` + `--dont-show-report` |
| 参数名版本差异 | `--cam`/`--time-calibration` 不识别 | 用 `--cams`；时延标定默认开启（要关才加 `--no-time-calibration`） |

## 3. 配置文件（kalibr_config/）

| 文件 | 要点 |
|---|---|
| `target_2uq2.yaml` | **targetRows/Cols = 内角数**（12x9 方格 → 8/11！写成方格数会 0 检测）；`rowSpacingMeters/colSpacingMeters` = 实测方格米数 |
| `camchain_2uq2.yaml` | pinhole+radtan 内参；`T_cn_cnm1` 必须 **4x4 矩阵嵌套列表**格式（7元素向量会报 invalid camera baseline） |
| `imu_2uq2.yaml` | `update_rate` 用 bag 实测值（`rostopic hz -b bag /imu0`）；噪声为初值，Kalibr calibrated 模型不细调 |

## 4. 录制标定 bag

```bash
# 终端1: 硬件模式启动(驱动+rosbridge)
cd ~/vision_imu/VINS-Fusion/docker
./run_foxglove.sh -d /root/catkin_ws/src/VINS-Fusion/config/2uq2/2uq2_stereo_imu_config.yaml

# 终端2: 录 bag(必须用挂载区路径, 容器删了也不丢)
docker exec -it vins-fusion bash
source /opt/ros/kinetic/setup.bash && source /root/catkin_ws/devel/setup.bash
rosbag record --lz4 -O /root/catkin_ws/src/VINS-Fusion/kalibr_config/data/calib_imu_cam.bag /cam0/image_raw /cam1/image_raw /imu0
```
录制要求（决定标定质量）：**30~60 秒、连续平滑六自由度运动**（慢速转+移，勿急晃/久停）、棋盘全程双眼完整可见、光照充足。

## 5. 播放 bag / Foxglove 观看

```bash
# 方式A(最简): Foxglove 直接打开 bag 文件
#   Open data source -> Local file -> 选 bag

# 方式B(带 rosbridge, 可同时看 VINS):
cd ~/vision_imu/VINS-Fusion/docker
./run_foxglove.sh /root/catkin_ws/src/VINS-Fusion/config/2uq2/2uq2_stereo_imu_config.yaml ~/vision_imu/VINS-Fusion
docker exec -it vins-fusion bash
source /opt/ros/kinetic/setup.bash && source /root/catkin_ws/devel/setup.bash
rosbag play /root/data/kalibr_config/data/calib_imu_cam.bag
```

### 只起 rosbridge（不重启容器）
```bash
# 当前容器内:
source /opt/ros/kinetic/setup.bash
roscore &                                   # 如未起
roslaunch rosbridge_server rosbridge_websocket.launch &
# 或宿主机注入:
docker exec -d <容器名> bash -c "source /opt/ros/kinetic/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch"
```

## 6. 运行标定

```bash
export MPLBACKEND=Agg
cd /root/catkin_ws/src/VINS-Fusion/kalibr_config

python /root/catkin_ws/src/VINS-Fusion/kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera \
  --target  target_2uq2.yaml \
  --cams    camchain_2uq2.yaml \
  --imu     imu_2uq2.yaml \
  --bag     /root/catkin_ws/src/VINS-Fusion/kalibr_config/data/calib_imu_cam.bag \
  --bag-freq 10.0 \
  --bag-from-to 0.0 30.0 \
  --dont-show-report
```
> 脚本不在 PATH：用完整路径或 `export PATH=$PATH:/root/catkin_ws/devel/lib/kalibr`。
> 输出文件以 bag 名为前缀写在**当前目录**（建议 cd 到 kalibr_config 再跑）。

## 7. 结果与转换

输出（`kalibr_config/results/`）：
- `calib_imu_cam-camchain-imucam.yaml`：`T_cam0_imu0`、`T_cam1_imu0`（imu→cam 变换）
- `calib_imu_cam-imu.yaml`：IMU 噪声参数
- `calib_imu_cam-report-imucam.pdf`：标定报告
- `calib_imu_cam-results-imucam.txt`：详细数值

**VINS 需要的 body_T_cam = inv(T_cam_imu)**（cam 系→IMU 系），转换方法：
```python
import numpy as np
T_cam_imu = np.array([...])   # 从 yaml 读 4x4
body_T_cam = np.linalg.inv(T_cam_imu)
```
填入 `config/2uq2/2uq2_stereo_imu_config.yaml` 的 body_T_cam0/1，并把 `estimate_extrinsic: 0`。

**本机标定结果要点**（2026-08 实机）：
- 重投影误差 cam0/cam1 = 0.21/0.23 px（优秀）
- 关键发现：IMU 与 cam0 轴系几乎**反向 180°**（diag ≈ -1）——之前 VINS 轨迹发散就是假设 IMU≈cam0 单位阵导致
- 时间偏移 -10.8ms（与驱动"IMU 打上一帧时间戳"吻合）
- IMU 噪声（VINS 参数）：acc_n 0.002 / gyr_n 0.0002 / acc_w 0.0002 / gyr_w 0.00002

## 8. 常见坑速查

| 现象 | 原因 | 解决 |
|---|---|---|
| 提取卡 Progress 19 | 多进程死锁 | IccSensors.py 强制单线程（见第2节） |
| 0 角点检测 | 棋盘行列数写成方格数 | 用内角数（12x9格→8/11） |
| invalid camera baseline | T_cn_cnm1 格式错 | 用 4x4 嵌套列表 |
| QXcbConnection 崩溃 | matplotlib 要显示器 | MPLBACKEND=Agg + --dont-show-report |
| Non-ASCII 报错 | 补丁注释用了中文 | Python2 文件注释用英文 |
| kalibr_xxx: command not found | 脚本不在 PATH | 用完整路径 devel/lib/kalibr/ |

## 9. A/B 实测结论（重要！）

**Kalibr 的 scale-misalignment 标度结果（在 66Hz IMU + 帧量化时间戳的 bag 上拟合）经实测不可靠**：
- 陀螺仪标度(÷1.06/1.05/1.02)、加速度计Y轴(÷0.958) 修正应用后，VINS 长路径漂移**明显变差**
- 关闭标度修正（用原始 4096 LSB/g、16.4 LSB/dps 换算）**明显更好**
- 结论：**低 IMU 频率 + 时间戳量化下，标度/时延/样条误差互相混淆，标度拟合过度**；原始换算更接近真实
- 驱动里 `imu_calib_enable` 默认 **false**（原始换算）；如需启用：`<param name="imu_calib_enable" value="true"/>`
- **外参（T_imu_cam，180° 旋转）是可靠的**（两次模型结果一致、残差极小），保留使用
- 要获得可靠标度，需用 **210fps 模式（~190Hz IMU）录 bag** 重新标定后再评估
