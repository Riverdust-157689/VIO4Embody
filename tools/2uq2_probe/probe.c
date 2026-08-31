/*
 * probe.c -- YLX-2UQ2 IMU 更新率探针
 *
 * 目的：实测 XU 扩展单元里 IMU 数据的真实更新频率。
 *   - 如果 IMU 序号每秒变化次数 ≈ 视频帧率  -> IMU 被锁在帧节奏上
 *   - 如果 ≈ 400~800Hz                       -> 可以按传感器速率取数，VINS 无忧
 *   - 如果 ≈ 轮询速度且序号连续              -> 数据按读取节奏更新
 *
 * 用法:
 *   gcc -o probe probe.c ./libSPV4L2XU.a
 *   sudo ./probe /dev/video2 [测试秒数]
 *
 * 说明: 实测该固件 XU 返回 15 字节 (3字节序号 + 12字节六轴, 无备用IMU组)。
 *   手册要求"必须先打开视频": 若序号一直不变(更新率≈0),
 *   请另开终端起一路视频流再测:
 *     v4l2-ctl -d /dev/video2 --set-fmt-video=width=3840,height=1080,pixelformat=MJPG
 *     v4l2-ctl -d /dev/video2 --stream-mmap --stream-loop
 */
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include "extunit.h"

#define UNIT_ID          3   /* 与官方 demo 一致 */
#define SELECTOR_IMU_DATA 1

static volatile int g_run = 1;
static void on_sig(int s) { (void)s; g_run = 0; }

int main(int argc, char **argv)
{
    const char *dev = (argc > 1) ? argv[1] : "/dev/video2";
    int seconds = (argc > 2) ? atoi(argv[2]) : 3;
    signal(SIGINT, on_sig);

    int fd = open(dev, O_RDWR);
    if (fd < 0) { perror("open 失败"); return 1; }

    unsigned long size = 0;
    if (xu_get_len(fd, UNIT_ID, SELECTOR_IMU_DATA, &size) != 0) {
        printf("xu_get_len 失败\n"); close(fd); return 2;
    }
    printf("设备 %s, XU数据长度 = %lu 字节\n", dev, size);
    /* 实测该固件返回 15 字节: 3字节序号 + 12字节六轴(无备用IMU组) */
    if (size < 3) {
        printf("!! 长度不足3字节，无法取序号。请确认设备节点 (v4l2-ctl --list-devices)\n");
        close(fd); return 3;
    }
    if (size < 15) {
        printf("!! 长度 %lu < 15，格式与预期不符，但序号在前3字节，仍尝试测量更新率\n", size);
    }

    unsigned char *buf = (unsigned char*)malloc(size);
    int first = 1;
    uint64_t reads = 0, unique = 0;
    uint32_t last_seq = 0;
    int have_last = 0;
    uint32_t min_delta = 0xFFFFFFFF, max_delta = 0;
    uint64_t sum_delta = 0; int deltas = 0;

    printf("开始高速轮询 %d 秒，测量 IMU 序号更新率...\n", seconds);
    time_t t0 = time(NULL), t1 = t0;
    while (g_run) {
        if (xu_get_cur(fd, UNIT_ID, SELECTOR_IMU_DATA, size, buf) != 0) {
            usleep(1000); continue;
        }
        uint32_t seq = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
        if (first) {
            first = 0;
            printf("\n首个数据包(前15字节): ");
            for (int i = 0; i < (int)(size < 15 ? size : 15); i++) printf("%02X ", buf[i]);
            printf("\n序号=%u, 若长度>=15: Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d (原始LSB)\n",
                   seq,
                   (int16_t)((buf[3]<<8)|buf[4]), (int16_t)((buf[5]<<8)|buf[6]), (int16_t)((buf[7]<<8)|buf[8]),
                   (int16_t)((buf[9]<<8)|buf[10]), (int16_t)((buf[11]<<8)|buf[12]), (int16_t)((buf[13]<<8)|buf[14]));
        }
        reads++;
        if (have_last) {
            if (seq != last_seq) {
                unique++;
                uint32_t d = (seq > last_seq) ? (seq - last_seq)
                                              : (seq + 0x1000000u - last_seq); /* 3字节回绕 */
                if (d < min_delta) min_delta = d;
                if (d > max_delta) max_delta = d;
                sum_delta += d; deltas++;
            }
        } else { have_last = 1; unique = 1; }
        last_seq = seq;
        t1 = time(NULL);
        if ((int)(t1 - t0) >= seconds) break;
    }

    double secs = (double)(t1 - t0);
    printf("\n===== 统计结果 (%.1f 秒) =====\n", secs);
    printf("XU 读取次数        : %llu  (%.1f 次/秒)\n",
           (unsigned long long)reads, secs > 0 ? reads/secs : 0);
    printf("序号发生变化次数  : %llu  (%.1f 次/秒)  <-- IMU 实际更新率\n",
           (unsigned long long)unique, secs > 0 ? unique/secs : 0);
    printf("序号未变(重复读取): %llu 次\n", (unsigned long long)(reads - unique));
    if (deltas) {
        printf("序号间隔: min=%u  max=%u  avg=%.2f\n",
               min_delta, max_delta, (double)sum_delta/deltas);
    }
    printf("\n判断:\n");
    printf("  更新率≈视频帧率(如60)  -> IMU 与帧锁同步, 需要和厂商确认能否提高\n");
    printf("  更新率≈400~800        -> 可按传感器速率取数, VINS 直接可用\n");
    printf("  更新率≈读取速度且连续  -> 数据按轮询节奏更新\n");

    free(buf); close(fd);
    return 0;
}
