cmake_minimum_required(VERSION 3.16)
project(imageBasics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 该示例既可由 ch5 顶层工程构建，也可在本目录中用 `cmake ..` 独立构建。
# OpenCV 4 的头文件位于 opencv4 子目录，必须通过 find_package 传入包含路径。
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imageBasics imageBasics.cpp)
# 链接OpenCV库
target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)
# 链接OpenCV库
target_link_libraries(undistortImage ${OpenCV_LIBS})
