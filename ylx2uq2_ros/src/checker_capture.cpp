/*
 * checker_capture.cpp -- 标定辅助: 双目配对采图
 *
 * 配对逻辑: 只有当左右相机在【同一帧时间戳】都检测到棋盘时, 才成对保存。
 *   这样保存的 cam0/cam1 图像是同一棋盘姿态, 后续可同时用于
 *   单目内参标定和双目外参(stereoCalibrate)标定。
 *
 * 两种模式:
 *   自动模式(默认): 双眼同时检测到就自动成对保存, 直到每目 target 张
 *   按键模式: keyboard_mode=true, 按 c/空格 请求一次采集(仍需双眼都检测到), q 退出
 *
 * 用法(与驱动同时运行):
 *   rosrun ylx2uq2_ros checker_capture
 *   rosrun ylx2uq2_ros checker_capture _keyboard_mode:=true
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <atomic>
#include <thread>
#include <map>
#include <mutex>
#include <cstdint>

static std::atomic<bool> g_capture_req0(false);   // 每相机独立按键标志
static std::atomic<bool> g_capture_req1(false);
static std::atomic<bool> g_quit(false);

// 递归创建目录
static void mkdir_p(const std::string &path)
{
    std::string cur;
    for (size_t i = 0; i <= path.size(); ++i) {
        if (i == path.size() || path[i] == '/') {
            if (!cur.empty())
                mkdir(cur.c_str(), 0755);
            if (i < path.size())
                cur += path[i];
        } else {
            cur += path[i];
        }
    }
}

// 键盘监听线程(原始模式): c/s/空格=采集请求(双眼), q=退出
static void key_thread()
{
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    while (!g_quit) {
        int c = getchar();
        if (c == 'c' || c == 's' || c == ' ') {
            g_capture_req0 = true;
            g_capture_req1 = true;
        }
        else if (c == 'q') {
            g_quit = true;
            break;
        }
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

class CheckerCapture
{
public:
    CheckerCapture(ros::NodeHandle &nh, bool keyboard_mode)
        : nh_(nh), keyboard_mode_(keyboard_mode)
    {
        pattern_w_    = nh_.param("pattern_w", 11);
        pattern_h_    = nh_.param("pattern_h", 8);
        target_       = nh_.param("target", 25);
        cam0_dir_     = nh_.param("cam0_dir", std::string("/root/catkin_ws/src/VINS-Fusion/config/2uq2/calib_data/cam0"));
        cam1_dir_     = nh_.param("cam1_dir", std::string("/root/catkin_ws/src/VINS-Fusion/config/2uq2/calib_data/cam1"));
        cam0_topic_   = nh_.param("cam0_topic", std::string("/cam0/image_raw"));
        cam1_topic_   = nh_.param("cam1_topic", std::string("/cam1/image_raw"));

        mkdir_p(cam0_dir_);
        mkdir_p(cam1_dir_);

        sub0_ = nh_.subscribe(cam0_topic_, 1, &CheckerCapture::cb0, this);
        sub1_ = nh_.subscribe(cam1_topic_, 1, &CheckerCapture::cb1, this);
        ROS_INFO("双目配对采图: 内角 %dx%d, 目标每目 %d 对 -> %s, %s",
                 pattern_w_, pattern_h_, target_, cam0_dir_.c_str(), cam1_dir_.c_str());
        if (keyboard_mode_)
            ROS_INFO("按键模式: 按 c/空格 采集(需双眼同时检测到棋盘), q 退出");
        else
            ROS_INFO("自动模式: 双眼同时检测到棋盘自动成对保存");
    }

    void cb0(const sensor_msgs::ImageConstPtr &msg) { recv0_++; handle(msg, cam0_dir_, n0_, g_capture_req0); }
    void cb1(const sensor_msgs::ImageConstPtr &msg) { recv1_++; handle(msg, cam1_dir_, n1_, g_capture_req1); }

private:
    struct Pending { cv::Mat img; std::string dir; int *cnt; };

    void handle(const sensor_msgs::ImageConstPtr &msg, const std::string &dir, int &count,
                std::atomic<bool> &cap_req)
    {
        if (keyboard_mode_) {
            if (!cap_req) return;         // 未按键
            cap_req = false;
        }
        cv::Mat img;
        try { img = cv_bridge::toCvCopy(msg, "mono8")->image; }
        catch (const std::exception &e) {
            ROS_WARN_THROTTLE(2.0, "[%s] cv_bridge 转换失败: %s (编码=%s 宽高=%dx%d)",
                              dir.c_str(), e.what(), msg->encoding.c_str(), msg->width, msg->height);
            return;
        }
        catch (...) {
            ROS_WARN_THROTTLE(2.0, "[%s] cv_bridge 转换失败(未知异常)", dir.c_str());
            return;
        }

        cv::Size pattern(pattern_w_, pattern_h_);
        std::vector<cv::Point2f> corners;
        bool ok = cv::findChessboardCorners(img, pattern, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS);
        if (!ok) {
            if (keyboard_mode_)
                ROS_WARN_THROTTLE(1.0, "[%s] 按键已按下但未检测到棋盘(或双眼未同步), 调整姿态再试", dir.c_str());
            return;
        }

        // ---------- 配对: 同一帧时间戳, 双眼都检测到才成对保存 ----------
        uint64_t key = msg->header.stamp.toNSec();
        std::lock_guard<std::mutex> lk(mtx_);
        cleanup_pending(key);
        auto it = pending_.find(key);
        if (it != pending_.end()) {
            // 对方已检测到同帧 -> 成对保存
            save(it->second.img, it->second.dir, it->second.cnt);
            save(img, dir, &count);
            pending_.erase(it);
        } else {
            pending_[key] = Pending{img, dir, &count};
        }

        // 每 5 秒汇报收包/保存统计
        ROS_INFO_THROTTLE(5.0, "[统计] cam0 收到 %d 帧(存 %d), cam1 收到 %d 帧(存 %d)",
                          recv0_, n0_, recv1_, n1_);
        if (n0_ >= target_ && n1_ >= target_)
            ROS_INFO_THROTTLE(5.0, "双目采图完成! 每目 %d 张", target_);
    }

    void cleanup_pending(uint64_t now)
    {
        // 超过 1 秒的未配对检测丢弃(相机同帧戳, 正常应毫秒级配对)
        while (!pending_.empty() && now - pending_.begin()->first > 1000000000ULL)
            pending_.erase(pending_.begin());
    }

    void save(const cv::Mat &img, const std::string &dir, int *cnt)
    {
        if (*cnt >= target_) return;
        char name[256];
        std::snprintf(name, sizeof(name), "%s/calib_%02d.jpg", dir.c_str(), *cnt);
        if (cv::imwrite(name, img)) {
            (*cnt)++;
            ROS_INFO("[%s] 已保存 %d/%d: %s", dir.c_str(), *cnt, target_, name);
        } else {
            ROS_WARN("[%s] 写入失败: %s", dir.c_str(), name);
        }
    }

    ros::NodeHandle &nh_;
    ros::Subscriber sub0_, sub1_;
    int pattern_w_, pattern_h_, target_, n0_ = 0, n1_ = 0;
    int recv0_ = 0, recv1_ = 0;
    bool keyboard_mode_;
    std::string cam0_dir_, cam1_dir_, cam0_topic_, cam1_topic_;
    std::mutex mtx_;
    std::map<uint64_t, Pending> pending_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "checker_capture");
    ros::NodeHandle nh("~");
    bool kb = nh.param("keyboard_mode", false);
    CheckerCapture cc(nh, kb);

    if (kb) {
        std::thread t(key_thread);
        ros::Rate r(10);
        while (ros::ok() && !g_quit) {
            ros::spinOnce();
            r.sleep();
        }
        if (!g_quit) g_quit = true;
        t.join();
    } else {
        ros::spin();
    }
    return 0;
}
