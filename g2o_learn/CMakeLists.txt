cmake_minimum_required(VERSION 3.16)

# 1. 项目名称
project(g2o_learning)

# 2. 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 3. 查找 Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 4. 查找 g2o
find_package(g2o REQUIRED CONFIG)

# 5. 定义公共依赖库
set(COMMON_LIBS
    Eigen3::Eigen
    g2o::core
    g2o::solver_dense
)

# 6. 创建多个可执行程序
foreach(target_name IN ITEMS 10)

    add_executable(${target_name} ${target_name}.cpp)

    target_link_libraries(${target_name}
        PRIVATE
        ${COMMON_LIBS}
    )

endforeach()