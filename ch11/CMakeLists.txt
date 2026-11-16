cmake_minimum_required(VERSION 3.10)
project(loop_closure LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 用户级依赖统一安装在 ~/.local；仍保留 /usr 和 /usr/local 的默认搜索。
if(DEFINED ENV{HOME})
    list(INSERT CMAKE_PREFIX_PATH 0 "$ENV{HOME}/.local")
endif()

find_package(OpenCV 4 REQUIRED)

# DBoW3 自带的旧版 Config 文件只暴露普通变量，且依赖全局
# link_directories。这里直接定位头文件和库，并封装成导入目标。
find_path(DBoW3_INCLUDE_DIR NAMES DBoW3/DBoW3.h)
find_library(DBoW3_LIBRARY NAMES DBoW3)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DBoW3 REQUIRED_VARS DBoW3_INCLUDE_DIR DBoW3_LIBRARY)
add_library(DBoW3::DBoW3 UNKNOWN IMPORTED)
set_target_properties(DBoW3::DBoW3 PROPERTIES
    IMPORTED_LOCATION "${DBoW3_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${DBoW3_INCLUDE_DIR}"
)

add_executable(feature_training feature_training.cpp)
add_executable(loop_closure loop_closure.cpp)
add_executable(gen_vocab gen_vocab_large.cpp)

foreach(target feature_training loop_closure gen_vocab)
    target_compile_features(${target} PRIVATE cxx_std_11)
    target_compile_options(${target} PRIVATE -O3)
    target_include_directories(${target} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE ${OpenCV_LIBS} DBoW3::DBoW3)
endforeach()
