#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Allan 方差分析: 从静止 IMU bag 估计 VINS 噪声参数
用法(容器内):
  python allan_analyze.py <imu_static.bag>
输出:
  acc_n / gyr_n : 噪声密度 (连续, 与采样率无关)
  acc_w / gyr_w : 偏置随机游走
"""
import sys
import numpy as np
import rosbag


def allan_dev(y, ms):
    """非重叠 Allan 标准差: AVAR(m) = 1/(2m^2(N-2m)) * sum((y_{i+2m}-2y_{i+m}+y_i)^2)"""
    N = len(y)
    taus, adev = [], []
    for m in ms:
        if 2 * m >= N:
            continue
        s = y[2 * m:] - 2.0 * y[m:N - m] + y[:N - 2 * m]
        adev.append(np.sqrt(np.mean(s ** 2) / (2.0 * m * m)))
        taus.append(m)
    return np.array(taus), np.array(adev)


def analyze(name, data, tau0, ms):
    taus, adev = allan_dev(data, ms)
    ok = adev > 0
    taus, adev = taus[ok], adev[ok]
    if len(taus) < 10:
        print("%s: 数据不足" % name)
        return None
    # 噪声密度: 白噪声区 ADEV(tau)=sigma*tau^(-1/2) -> sigma=ADEV*sqrt(tau)
    noise = np.median(adev[:6] * np.sqrt(taus[:6]))
    # 偏置随机游走: 大tau区 ADEV(tau)=K*sqrt(tau/3) -> K=ADEV*sqrt(3)/sqrt(tau)
    rw = np.median(adev[-6:] * np.sqrt(3.0) / np.sqrt(taus[-6:]))
    bias = float(np.min(adev))
    print("%-8s: noise_density=%.6g   random_walk=%.6g   bias_instab=%.6g"
          % (name, noise, rw, bias))
    return noise, rw


def main():
    bagfile = sys.argv[1] if len(sys.argv) > 1 else \
        "/root/catkin_ws/src/VINS-Fusion/kalibr_config/data/imu_static.bag"
    bag = rosbag.Bag(bagfile)
    ts, acc, gyr = [], [], []
    for _, msg, _ in bag.read_messages(topics=['/imu0']):
        ts.append(msg.header.stamp.to_sec())
        acc.append([msg.linear_acceleration.x, msg.linear_acceleration.y,
                    msg.linear_acceleration.z])
        gyr.append([msg.angular_velocity.x, msg.angular_velocity.y,
                    msg.angular_velocity.z])
    bag.close()
    ts = np.array(ts)
    acc = np.array(acc)
    gyr = np.array(gyr)
    if len(ts) < 500:
        print("IMU 样本太少: %d, 请录更长时间" % len(ts))
        return
    dt = float(np.median(np.diff(ts)))
    print("样本: %d  时长: %.1fs  采样间隔: %.4fs (%.1f Hz)"
          % (len(ts), ts[-1] - ts[0], dt, 1.0 / dt))
    # 去均值(偏置不影响 Allan 的噪声/游走估计, 但帮助数值稳定)
    acc -= acc.mean(axis=0)
    gyr -= gyr.mean(axis=0)

    N = len(ts)
    ms = np.unique(np.logspace(0, np.log10(min(N // 4, 20000)), 100).astype(int))
    ms = ms[ms >= 1]

    print("\n=== 陀螺仪 (rad/s) ===")
    gyr_res = [analyze("gyr_%s" % c, gyr[:, i], dt, ms) for i, c in enumerate('xyz')]
    print("\n=== 加速度计 (m/s^2) ===")
    acc_res = [analyze("acc_%s" % c, acc[:, i], dt, ms) for i, c in enumerate('xyz')]

    gyr_n = max(r[0] for r in gyr_res if r)
    gyr_w = max(r[1] for r in gyr_res if r)
    acc_n = max(r[0] for r in acc_res if r)
    acc_w = max(r[1] for r in acc_res if r)

    print("""
================ 建议填入 VINS 配置(取各轴较大值) ================
  acc_n: %.6g     # 加速度计噪声密度 (m/s^2/sqrt(Hz))
  gyr_n: %.6g     # 陀螺仪噪声密度 (rad/s/sqrt(Hz))
  acc_w: %.6g     # 加速度计偏置随机游走 (m/s^2/sqrt(s))
  gyr_w: %.6g     # 陀螺仪偏置随机游走 (rad/s/sqrt(s))
==================================================================
    """ % (acc_n, gyr_n, acc_w, gyr_w))


if __name__ == '__main__':
    main()
