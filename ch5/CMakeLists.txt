cmake_minimum_required(VERSION 3.16)
project(slambook2_ch5 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 添加 C++11 标准支持
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Eigen
include_directories("/usr/include/eigen3")

# 教学代码统一使用 OpenCV 4；组件列表与原程序一致，不改变图像处理逻辑。
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui calib3d)
message(STATUS "Using OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(imageBasics)
add_subdirectory(stereo)
add_subdirectory(rgbd)
