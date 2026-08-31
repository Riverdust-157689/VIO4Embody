/*
 * stereo_extrinsics.cpp -- 双目外参标定工具
 *
 * 用配对棋盘图像 + 已标定的两个 pinhole 内参, 计算 cam0<->cam1 相对位姿
 * (OpenCV stereoCalibrate, 固定内参只优化外参)。
 *
 * 编译(容器内):
 *   g++ -O2 -o stereo_extrinsics stereo_extrinsics.cpp $(pkg-config --cflags --libs opencv)
 *
 * 用法:
 *   ./stereo_extrinsics <cam0图目录> <cam1图目录> \
 *       <cam0内参yaml> <cam1内参yaml> <方格尺寸mm> [配对数量,默认25]
 *
 * 例:
 *   ./stereo_extrinsics \
 *     /root/catkin_ws/src/VINS-Fusion/config/2uq2/calib_data/cam0 \
 *     /root/catkin_ws/src/VINS-Fusion/config/2uq2/calib_data/cam1 \
 *     /root/catkin_ws/src/VINS-Fusion/config/2uq2/cam0_pinhole.yaml \
 *     /root/catkin_ws/src/VINS-Fusion/config/2uq2/cam1_pinhole.yaml \
 *     28.66
 *
 * 约定: OpenCV stereoCalibrate 返回的 R,T 满足  X_cam1 = R * X_cam0 + T
 *       (cam0 坐标系 -> cam1 坐标系)。
 * 输出同时给出 VINS 配置可直接粘贴的 body_T_cam0/body_T_cam1 数据块
 * (假设 IMU 坐标系≈cam0 坐标系, VINS estimate_extrinsic=1 会在线精调)。
 */
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

static bool load_pinhole(const std::string &path, cv::Mat &K, cv::Mat &D)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        fprintf(stderr, "无法打开内参文件: %s\n", path.c_str());
        return false;
    }
    double fx, fy, cx, cy, k1, k2, p1, p2;
    fs["projection_parameters"]["fx"] >> fx;
    fs["projection_parameters"]["fy"] >> fy;
    fs["projection_parameters"]["cx"] >> cx;
    fs["projection_parameters"]["cy"] >> cy;
    fs["distortion_parameters"]["k1"] >> k1;
    fs["distortion_parameters"]["k2"] >> k2;
    fs["distortion_parameters"]["p1"] >> p1;
    fs["distortion_parameters"]["p2"] >> p2;
    K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    D = (cv::Mat_<double>(1, 4) << k1, k2, p1, p2);
    printf("内参 %s: fx=%.2f fy=%.2f cx=%.2f cy=%.2f k1=%.5f k2=%.5f p1=%.6f p2=%.6f\n",
           path.c_str(), fx, fy, cx, cy, k1, k2, p1, p2);
    return true;
}

static void print_yaml_block(const char *name, const cv::Mat &T4)
{
    printf("\n%s: !!opencv-matrix\n", name);
    printf("   rows: 4\n   cols: 4\n   dt: d\n   data: [");
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            printf("%.10g%s", T4.at<double>(r, c),
                   (r == 3 && c == 3) ? "]\n" : ", ");
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 6) {
        printf("用法: %s <cam0图目录> <cam1图目录> <cam0内参yaml> <cam1内参yaml> <方格mm> [配对数量]\n", argv[0]);
        return 1;
    }
    std::string dir0 = argv[1], dir1 = argv[2];
    std::string y0 = argv[3], y1 = argv[4];
    double sq = atof(argv[5]);
    int max_pairs = argc > 6 ? atoi(argv[6]) : 25;

    cv::Mat K0, D0, K1, D1;
    if (!load_pinhole(y0, K0, D0)) return 1;
    if (!load_pinhole(y1, K1, D1)) return 1;

    cv::Size pattern(11, 8);   // 12x9 棋盘 -> 内角 11x8
    std::vector<cv::Point3f> obj;
    for (int r = 0; r < pattern.height; r++)
        for (int c = 0; c < pattern.width; c++)
            obj.push_back(cv::Point3f((float)(c * sq), (float)(r * sq), 0));

    std::vector<std::vector<cv::Point3f>> obj_pts;
    std::vector<std::vector<cv::Point2f>> img_pts0, img_pts1;
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS;
    int used = 0;
    for (int i = 0; i < max_pairs; i++) {
        char p0[512], p1[512];
        snprintf(p0, sizeof(p0), "%s/calib_%02d.jpg", dir0.c_str(), i);
        snprintf(p1, sizeof(p1), "%s/calib_%02d.jpg", dir1.c_str(), i);
        cv::Mat im0 = cv::imread(p0, cv::IMREAD_GRAYSCALE);
        cv::Mat im1 = cv::imread(p1, cv::IMREAD_GRAYSCALE);
        if (im0.empty() || im1.empty()) continue;
        std::vector<cv::Point2f> c0, c1;
        bool ok0 = cv::findChessboardCorners(im0, pattern, c0, flags);
        bool ok1 = cv::findChessboardCorners(im1, pattern, c1, flags);
        if (!ok0 || !ok1) {
            printf("跳过配对 %d: cam0=%s cam1=%s\n", i, ok0 ? "OK" : "未检测", ok1 ? "OK" : "未检测");
            continue;
        }
        cv::cornerSubPix(im0, c0, cv::Size(5, 5), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
        cv::cornerSubPix(im1, c1, cv::Size(5, 5), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
        obj_pts.push_back(obj);
        img_pts0.push_back(c0);
        img_pts1.push_back(c1);
        used++;
    }
    printf("有效配对: %d 对\n", used);
    if (used < 10) {
        printf("配对数量不足(<10), 请重新采图\n");
        return 1;
    }

    cv::Mat R, T, E, F;
    double rms = cv::stereoCalibrate(obj_pts, img_pts0, img_pts1, K0, D0, K1, D1,
                                     cv::Size(1280, 720), R, T, E, F,
                                     cv::CALIB_FIX_INTRINSIC);
    printf("\n===== 双目外参结果 =====\n");
    printf("stereoCalibrate RMS: %.3f px\n", rms);
    printf("\nR (cam0->cam1):\n");
    for (int r = 0; r < 3; r++)
        printf("  [%10.6f %10.6f %10.6f]\n", R.at<double>(r, 0), R.at<double>(r, 1), R.at<double>(r, 2));
    printf("T (cam0->cam1, mm): [%.2f, %.2f, %.2f]\n",
           T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0));
    double baseline = cv::norm(T);
    printf("基线长度: %.1f mm  (常规双目 30~80mm, 可据此判断结果是否合理)\n", baseline);

    // VINS 配置块: 假设 IMU 坐标系≈cam0, 则 body_T_cam0=单位阵, body_T_cam1=T_cam0_cam1
    cv::Mat T01 = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T01(cv::Rect(0, 0, 3, 3)));
    T.copyTo(T01(cv::Rect(3, 0, 1, 3)));
    print_yaml_block("body_T_cam0 (假设IMU≈cam0)", cv::Mat::eye(4, 4, CV_64F));
    print_yaml_block("body_T_cam1 (若轨迹异常可换用逆变换试)", T01);

    printf("\n提示: 若 VINS 跑起来轨迹左右/前后镜像或基线符号不对,\n");
    printf("      把 body_T_cam1 换成 T01 的逆矩阵再试(estimate_extrinsic=1 会自动精调)。\n");
    return 0;
}
