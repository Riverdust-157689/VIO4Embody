/*
 * probe_switch.c -- 读写 XU 单字节选择器(如外触发开关), 验证是否可写及语义
 *
 * 用法:
 *   gcc -o probe_switch probe_switch.c ./libSPV4L2XU.a
 *   sudo ./probe_switch /dev/video0 <selector> [写值]
 *   例: sudo ./probe_switch /dev/video0 2      # 读 selector 2
 *       sudo ./probe_switch /dev/video0 2 0    # 写 selector 2 = 0
 * 注意: 若 selector 是外触发开关, 写 0 = 常规视频流模式(手册), 先起视频流再试
 */
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "extunit.h"

#define UNIT_ID 3

int main(int argc, char **argv)
{
    if (argc < 3) {
        printf("用法: %s <设备> <selector> [写值]\n", argv[0]);
        return 1;
    }
    const char *dev = argv[1];
    int sel = atoi(argv[2]);
    int write_val = (argc > 3) ? atoi(argv[3]) : -1;

    int fd = open(dev, O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    unsigned long len = 0;
    if (xu_get_len(fd, UNIT_ID, sel, &len) != 0 || len < 1) {
        printf("selector %d 不可用\n", sel);
        close(fd);
        return 1;
    }
    printf("selector %d 长度 %lu\n", sel, len);

    if (write_val >= 0) {
        unsigned char w = (unsigned char)write_val;
        int r = xu_set_cur(fd, UNIT_ID, sel, 1, &w);
        printf("写入 %d: %s\n", write_val, r == 0 ? "成功" : "失败");
    }

    unsigned char v = 0;
    if (xu_get_cur(fd, UNIT_ID, sel, 1, &v) == 0)
        printf("读取当前值: %d\n", (int)v);
    else
        printf("读取失败\n");

    close(fd);
    return 0;
}
