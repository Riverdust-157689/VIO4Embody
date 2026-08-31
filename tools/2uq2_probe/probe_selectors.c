/*
 * probe_selectors.c -- 枚举 UVC XU 扩展单元的所有选择器
 *
 * 目的: 查找可能存在的"高频 IMU 通道"或"批量数据"选择器
 *       (当前已知 selector=1 是帧锁 IMU, 也许有别的选择器藏了高频数据)
 *
 * 用法:
 *   gcc -o probe_selectors probe_selectors.c ./libSPV4L2XU.a
 *   sudo ./probe_selectors /dev/video2 [unit_id] [最大selector]
 * 注意: 先启动视频流
 */
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "extunit.h"

int main(int argc, char **argv)
{
    const char *dev = (argc > 1) ? argv[1] : "/dev/video2";
    int unit = (argc > 2) ? atoi(argv[2]) : 3;
    int max_sel = (argc > 3) ? atoi(argv[3]) : 64;

    int fd = open(dev, O_RDWR);
    if (fd < 0) { perror("open 失败"); return 1; }

    printf("设备 %s, 单元ID %d, 枚举选择器 0..%d:\n", dev, unit, max_sel);
    for (int sel = 0; sel <= max_sel; sel++) {
        unsigned long len = 0;
        if (xu_get_len(fd, unit, sel, &len) == 0) {
            printf("  selector %2d : 长度 %lu 字节", sel, len);
            if (len > 0) {
                unsigned char tmp[512];
                unsigned long rlen = len < sizeof(tmp) ? len : sizeof(tmp);
                if (xu_get_cur(fd, unit, sel, rlen, tmp) == 0) {
                    printf("  前%d字节: ", (int)rlen);
                    for (unsigned long i = 0; i < (rlen < 8 ? rlen : 8); i++)
                        printf("%02X ", tmp[i]);
                }
            }
            printf("\n");
        }
    }
    close(fd);
    return 0;
}
