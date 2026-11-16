cmake_minimum_required(VERSION 3.10)
project(ch12 LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 让两个子项目都能发现安装在用户目录下的 Sophus、OctoMap 等依赖。
if(DEFINED ENV{HOME})
    list(INSERT CMAKE_PREFIX_PATH 0 "$ENV{HOME}/.local")
endif()

add_subdirectory(dense_mono)
add_subdirectory(dense_RGBD)
