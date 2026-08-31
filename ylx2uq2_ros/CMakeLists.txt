cmake_minimum_required(VERSION 2.8.3)
project(ylx2uq2_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ylx2uq2_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libSPV4L2XU.a
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 标定辅助: 棋盘格采图
add_executable(checker_capture src/checker_capture.cpp)
target_link_libraries(checker_capture
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(TARGETS checker_capture
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
