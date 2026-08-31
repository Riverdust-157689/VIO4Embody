/*
 * probe_content.c -- 验证 IMU"序号锁帧但内容高频更新"猜想
 *
 * 高速轮询 XU, 分别统计:
 *   序号变化率  -> 帧锁节奏(视频帧率)
 *   内容变化率  -> 真实 IMU 采样节奏(如果 >> 序号变化率, 说明每帧间有多个 IMU 样本!)
 *
 * 用法:
 *   gcc -o probe_content probe_content.c ./libSPV4L2XU.a
 *   sudo ./probe_content /dev/video2 [秒数]
 * 注意: 必须先启动视频流(另开终端: v4l2-ctl -d /dev/video2 --stream-mmap --stream-loop)
 */
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "extunit.h"

#define UNIT_ID          3
#define SELECTOR_IMU_DATA 1

/* FNV-1a 64位哈希, 用 12 字节 IMU 内容(6轴x2字节)做指纹 */
static uint64_t content_fp(const unsigned char *b)
{
    uint64_t h = 1469598103934665603ULL;
    for (int i = 0; i < 12; i++) { h ^= b[i]; h *= 1099511628211ULL; }
    return h;
}

int main(int argc, char **argv)
{
    const char *dev = (argc > 1) ? argv[1] : "/dev/video2";
    int seconds = (argc > 2) ? atoi(argv[2]) : 5;

    int fd = open(dev, O_RDWR);
    if (fd < 0) { perror("open 失败"); return 1; }

    unsigned long len = 0;
    if (xu_get_len(fd, UNIT_ID, SELECTOR_IMU_DATA, &len) != 0 || len < 15) {
        printf("XU 失败或长度 %lu < 15。请先启动视频流:\n", len);
        printf("  v4l2-ctl -d /dev/video2 --stream-mmap --stream-loop\n");
        return 2;
    }
    printf("设备 %s, XU长度 %lu 字节, 轮询 %d 秒...\n", dev, len, seconds);

    unsigned char *buf = (unsigned char*)malloc(len);
    uint64_t reads = 0, seq_changes = 0, content_changes = 0;
    uint32_t last_seq = 0;
    uint64_t last_fp = 0;
    int have = 0;
    int cur_cnt = 1;                  // 当前序号下出现过的不同内容数
    uint64_t sum_same_seq_distinct = 0, max_same_seq_distinct = 0;
    uint64_t seq_groups = 0;

    time_t t0 = time(NULL), t1 = t0;
    while ((int)(t1 - t0) < seconds) {
        if (xu_get_cur(fd, UNIT_ID, SELECTOR_IMU_DATA, len, buf) != 0) {
            usleep(500);
            continue;
        }
        uint32_t seq = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
        uint64_t fp = content_fp(buf + 3);
        reads++;
        if (!have) {
            have = 1; last_seq = seq; last_fp = fp; cur_cnt = 1;
        } else {
            if (seq != last_seq) {
                seq_changes++;
                if (cur_cnt > 1) {
                    sum_same_seq_distinct += cur_cnt;
                    if ((uint64_t)cur_cnt > max_same_seq_distinct)
                        max_same_seq_distinct = cur_cnt;
                    seq_groups++;
                }
                cur_cnt = 1;
                last_seq = seq;
            } else if (fp != last_fp) {
                cur_cnt++;
            }
            if (fp != last_fp)
                content_changes++;
            last_fp = fp;
        }
        t1 = time(NULL);
    }

    double secs = (double)(t1 - t0);
    printf("\n===== 结果 (%.1f 秒) =====\n", secs);
    printf("XU 读取次数     : %llu (%.1f 次/秒)  <-- 轮询上限\n",
           (unsigned long long)reads, reads / secs);
    printf("序号变化次数     : %llu (%.1f 次/秒)  <-- 帧锁节奏\n",
           (unsigned long long)seq_changes, seq_changes / secs);
    printf("内容变化次数     : %llu (%.1f 次/秒)  <-- 真实IMU采样节奏\n",
           (unsigned long long)content_changes, content_changes / secs);
    if (seq_groups)
        printf("同一序号内不同内容: 平均 %.1f 个, 最大 %llu 个\n",
               sum_same_seq_distinct / (double)seq_groups, max_same_seq_distinct);
    printf("\n判断: 内容变化率明显高于序号变化率 -> 每帧间有多个 IMU 样本, 可高频采集!\n");
    printf("      内容变化率≈序号变化率      -> IMU 确实锁在帧率上\n");

    free(buf);
    close(fd);
    return 0;
}
