cmake_minimum_required(VERSION 2.8)
project(stereo_extrinsics)

# GCC 5.4 默认 C++98, 显式启用 C++11
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

add_executable(stereo_extrinsics stereo_extrinsics.cpp)
target_link_libraries(stereo_extrinsics ${OpenCV_LIBS})
