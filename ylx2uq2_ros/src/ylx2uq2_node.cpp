/*
 * ylx2uq2_node.cpp -- YLX-2UQ2 双目全局快门相机 + 内置 IMU 的 ROS1 驱动
 *
 * 功能:
 *  1. V4L2 采集 MJPG 双画面 (如 2560x720 = 左右各 1280x720)
 *  2. OpenCV 解码 + 双目切分 -> /cam0/image_raw, /cam1/image_raw
 *  3. 图像抽帧 (image_subsample) 降低 VINS 处理负载, IMU 不受影响
 *  4. UVC XU 读取内置 IMU (15字节: 3序号 + 6轴x2字节大端) -> /imu0
 *     IMU 与视频帧锁同步, 用当前视频帧的 V4L2 时间戳给 IMU 打戳
 *  5. 启动时自动设置 分辨率/帧率/曝光/增益
 *
 * 话题与 VINS 的 euRoC 配置一致: /cam0/image_raw /cam1/image_raw /imu0
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <vector>

#include "ylx2uq2_ros/extunit.h"

struct V4L2Buffer { void *start; size_t length; };

// Kalibr scale-misalignment 标定矩阵的逆 (2026-08-31, bag: calib_imu_cam.bag)
// true = M_INV * measured
static const double M_ACC_INV[3][3] = {
    {1.00517225, 0, 0},
    {0.0025328281, 1.04326813, 0},
    {-0.00153371781, 0.00708191792, 1.0003441},
};
static const double M_GYRO_INV[3][3] = {
    {0.941224998, 0, 0},
    {-0.0266349857, 0.953396301, 0},
    {0.0159201106, -0.00756498534, 0.982198034},
};

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ylx2uq2_driver");
    ros::NodeHandle nh("~");

    // ==================== 参数 ====================
    std::string device      = nh.param("device", std::string("/dev/video2"));
    int width               = nh.param("width", 2560);
    int height              = nh.param("height", 720);
    int fps                 = nh.param("fps", 120);
    int subsample           = nh.param("image_subsample", 4);   // 1=全发, 4=每4帧发1帧
    bool swap_cams          = nh.param("swap_cams", false);     // 左右互换(若发现反了)
    int exposure_auto       = nh.param("exposure_auto", 1);     // 1=手动, 3=自动
    int exposure_abs        = nh.param("exposure_abs", 50);     // 单位100us
    int gain                = nh.param("gain", 80);
    int xu_unit             = nh.param("xu_unit", 3);
    int xu_selector         = nh.param("xu_selector", 1);
    std::string cam0_topic  = nh.param("cam0_topic", std::string("/cam0/image_raw"));
    std::string cam1_topic  = nh.param("cam1_topic", std::string("/cam1/image_raw"));
    std::string imu_topic   = nh.param("imu_topic", std::string("/imu0"));
    std::string frame_id    = nh.param("frame_id", std::string("camera"));
    // Kalibr scale-misalignment 标度修正开关 (A/B 对比用; 若加了反而更飘可关掉)
    bool imu_calib_enable   = nh.param("imu_calib_enable", false);  // 默认关闭: A/B实测原始换算更稳

    ros::Publisher pub_cam0 = nh.advertise<sensor_msgs::Image>(cam0_topic, 5);
    ros::Publisher pub_cam1 = nh.advertise<sensor_msgs::Image>(cam1_topic, 5);
    ros::Publisher pub_imu  = nh.advertise<sensor_msgs::Imu>(imu_topic, 300);

    // ==================== 打开设备 ====================
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        ROS_FATAL("打开 %s 失败: %s (检查 --device 透传)", device.c_str(), strerror(errno));
        return 1;
    }

    // ==================== 设置格式 ====================
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        ROS_FATAL("VIDIOC_S_FMT 失败: %s", strerror(errno));
        return 1;
    }
    width  = fmt.fmt.pix.width;
    height = fmt.fmt.pix.height;
    ROS_INFO("视频格式: %dx%d MJPG", width, height);

    // ==================== 设置帧率 ====================
    if (fps > 0) {
        struct v4l2_streamparm parm;
        memset(&parm, 0, sizeof(parm));
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator   = 1;
        parm.parm.capture.timeperframe.denominator = fps;
        if (xioctl(fd, VIDIOC_S_PARM, &parm) < 0)
            ROS_WARN("VIDIOC_S_PARM(%d fps) 失败: %s", fps, strerror(errno));
        else
            ROS_INFO("请求帧率: %d fps (实际以固件为准)", fps);
        // 回读实际协商的帧率
        struct v4l2_streamparm qparm;
        memset(&qparm, 0, sizeof(qparm));
        qparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_G_PARM, &qparm) == 0 && qparm.parm.capture.timeperframe.numerator > 0) {
            double actual = qparm.parm.capture.timeperframe.denominator /
                            (double)qparm.parm.capture.timeperframe.numerator;
            ROS_INFO("协商后帧率: %.1f fps", actual);
        }
    }

    // ==================== 曝光/增益 ====================
    if (exposure_auto >= 0) {
        struct v4l2_control c; memset(&c, 0, sizeof(c));
        c.id = V4L2_CID_EXPOSURE_AUTO; c.value = exposure_auto;
        if (xioctl(fd, VIDIOC_S_CTRL, &c) < 0)
            ROS_WARN("设置 exposure_auto 失败: %s", strerror(errno));
        // 回读
        struct v4l2_control g; memset(&g, 0, sizeof(g));
        g.id = V4L2_CID_EXPOSURE_AUTO;
        if (xioctl(fd, VIDIOC_G_CTRL, &g) == 0)
            ROS_INFO("exposure_auto 当前值: %ld (1=手动 3=自动)", (long)g.value);
    }
    if (exposure_abs > 0) {
        struct v4l2_control c; memset(&c, 0, sizeof(c));
        c.id = V4L2_CID_EXPOSURE_ABSOLUTE; c.value = exposure_abs;
        if (xioctl(fd, VIDIOC_S_CTRL, &c) < 0)
            ROS_WARN("设置 exposure_abs 失败: %s", strerror(errno));
        struct v4l2_control g; memset(&g, 0, sizeof(g));
        g.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        if (xioctl(fd, VIDIOC_G_CTRL, &g) == 0)
            ROS_INFO("exposure_abs 当前值: %ld", (long)g.value);
    }
    if (gain > 0) {
        struct v4l2_control c; memset(&c, 0, sizeof(c));
        c.id = V4L2_CID_GAIN; c.value = gain;
        if (xioctl(fd, VIDIOC_S_CTRL, &c) < 0)
            ROS_WARN("设置 gain 失败: %s", strerror(errno));
    }

    // ==================== mmap 缓冲 ====================
    struct v4l2_requestbuffers reqbuf;
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.count  = 4;
    reqbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &reqbuf) < 0) {
        ROS_FATAL("VIDIOC_REQBUFS: %s", strerror(errno));
        return 1;
    }
    std::vector<V4L2Buffer> buffers(reqbuf.count);
    for (unsigned i = 0; i < reqbuf.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            ROS_FATAL("VIDIOC_QUERYBUF: %s", strerror(errno));
            return 1;
        }
        buffers[i].length = buf.length;
        buffers[i].start  = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffers[i].start == MAP_FAILED) {
            ROS_FATAL("mmap: %s", strerror(errno));
            return 1;
        }
        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            ROS_FATAL("VIDIOC_QBUF: %s", strerror(errno));
            return 1;
        }
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        ROS_FATAL("VIDIOC_STREAMON: %s", strerror(errno));
        return 1;
    }
    ROS_INFO("视频流已启动");

    // ==================== XU IMU 初始化 ====================
    unsigned long imu_len = 0;
    if (xu_get_len(fd, xu_unit, xu_selector, &imu_len) != 0 || imu_len < 3) {
        ROS_FATAL("xu_get_len 失败, len=%lu (设备=%s 单元=%d 选择器=%d)",
                  imu_len, device.c_str(), xu_unit, xu_selector);
        return 1;
    }
    ROS_INFO("IMU 报文长度: %lu 字节", imu_len);
    std::vector<unsigned char> imu_buf(imu_len);
    uint32_t last_seq = 0;
    bool have_seq = false;
    int frame_cnt = 0;
    ros::Time prev_ts_;
    bool has_prev_ts_ = false;

    std::vector<uchar> jpeg;
    cv::Mat frame;

    // ==================== 主循环 ====================
    while (ros::ok()) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) continue;
            ROS_WARN("VIDIOC_DQBUF: %s", strerror(errno));
            continue;
        }
        // V4L2 时间戳(单调时钟), 图像与 IMU 共用, 保证相对时序一致
        ros::Time ts(buf.timestamp.tv_sec, buf.timestamp.tv_usec * 1000);
        // XU 寄存器里的 IMU 样本属于上一帧(帧锁, 帧捕获时更新), 用上一帧时间戳打点
        ros::Time imu_ts = has_prev_ts_ ? prev_ts_ : ts;
        prev_ts_ = ts;
        has_prev_ts_ = true;

        // ---------- 图像: 解码 + 切分 + 抽帧 ----------
        frame_cnt++;
        if (frame_cnt % subsample == 0) {
            jpeg.assign(static_cast<uchar*>(buffers[buf.index].start),
                        static_cast<uchar*>(buffers[buf.index].start) + buf.bytesused);
            frame = cv::imdecode(jpeg, cv::IMREAD_COLOR);
            if (!frame.empty() && frame.cols >= 2) {
                int half = frame.cols / 2;
                cv::Mat l = frame(cv::Rect(0,    0, half, frame.rows)).clone();
                cv::Mat r = frame(cv::Rect(half, 0, frame.cols - half, frame.rows)).clone();
                sensor_msgs::ImagePtr msg0 = cv_bridge::CvImage(
                    std_msgs::Header(), "bgr8", swap_cams ? r : l).toImageMsg();
                sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(
                    std_msgs::Header(), "bgr8", swap_cams ? l : r).toImageMsg();
                msg0->header.stamp = ts; msg0->header.frame_id = frame_id;
                msg1->header.stamp = ts; msg1->header.frame_id = frame_id;
                pub_cam0.publish(msg0);
                pub_cam1.publish(msg1);
            } else {
                ROS_WARN_THROTTLE(2.0, "MJPG 解码失败");
            }
        }

        // ---------- IMU: 每帧读一次 XU, 序号变化才发 ----------
        if (xu_get_cur(fd, xu_unit, xu_selector, imu_len, imu_buf.data()) == 0 && imu_len >= 15) {
            uint32_t seq = (imu_buf[0] << 16) | (imu_buf[1] << 8) | imu_buf[2];
            if (!have_seq || seq != last_seq) {
                have_seq = true;
                last_seq = seq;
                int16_t ax = (int16_t)((imu_buf[3]  << 8) | imu_buf[4]);
                int16_t ay = (int16_t)((imu_buf[5]  << 8) | imu_buf[6]);
                int16_t az = (int16_t)((imu_buf[7]  << 8) | imu_buf[8]);
                int16_t gx = (int16_t)((imu_buf[9]  << 8) | imu_buf[10]);
                int16_t gy = (int16_t)((imu_buf[11] << 8) | imu_buf[12]);
                int16_t gz = (int16_t)((imu_buf[13] << 8) | imu_buf[14]);

                sensor_msgs::Imu msg;
                msg.header.stamp    = imu_ts;   // 上一帧时间戳(样本实际属于上一帧)
                msg.header.frame_id = frame_id;
                // 原始换算: 加速度 LSB/4096 g -> m/s^2 ; 陀螺仪 LSB/16.4 dps -> rad/s
                double am[3] = { ax / 4096.0 * 9.80665,
                                 ay / 4096.0 * 9.80665,
                                 az / 4096.0 * 9.80665 };
                double gm[3] = { gx / 16.4 * M_PI / 180.0,
                                 gy / 16.4 * M_PI / 180.0,
                                 gz / 16.4 * M_PI / 180.0 };
                // Kalibr scale-misalignment 校正: true = inv(M) * measured
                // (2026-08-31 标定: 陀螺仪标度偏高2-6%, 加速度计Y轴偏低4%)
                if (imu_calib_enable) {
                    msg.linear_acceleration.x = M_ACC_INV[0][0]*am[0] + M_ACC_INV[0][1]*am[1] + M_ACC_INV[0][2]*am[2];
                    msg.linear_acceleration.y = M_ACC_INV[1][0]*am[0] + M_ACC_INV[1][1]*am[1] + M_ACC_INV[1][2]*am[2];
                    msg.linear_acceleration.z = M_ACC_INV[2][0]*am[0] + M_ACC_INV[2][1]*am[1] + M_ACC_INV[2][2]*am[2];
                    msg.angular_velocity.x    = M_GYRO_INV[0][0]*gm[0] + M_GYRO_INV[0][1]*gm[1] + M_GYRO_INV[0][2]*gm[2];
                    msg.angular_velocity.y    = M_GYRO_INV[1][0]*gm[0] + M_GYRO_INV[1][1]*gm[1] + M_GYRO_INV[1][2]*gm[2];
                    msg.angular_velocity.z    = M_GYRO_INV[2][0]*gm[0] + M_GYRO_INV[2][1]*gm[1] + M_GYRO_INV[2][2]*gm[2];
                } else {
                    msg.linear_acceleration.x = am[0];
                    msg.linear_acceleration.y = am[1];
                    msg.linear_acceleration.z = am[2];
                    msg.angular_velocity.x    = gm[0];
                    msg.angular_velocity.y    = gm[1];
                    msg.angular_velocity.z    = gm[2];
                }
                pub_imu.publish(msg);
            }
        }

        xioctl(fd, VIDIOC_QBUF, &buf);
    }

    // ==================== 清理 ====================
    xioctl(fd, VIDIOC_STREAMOFF, &type);
    for (unsigned i = 0; i < buffers.size(); i++)
        munmap(buffers[i].start, buffers[i].length);
    close(fd);
    ROS_INFO("驱动退出");
    return 0;
}
