# SLAM Learning State

## 当前进度

- 当前章节：Chapter 3（三维空间刚体的运动）
- 当前阶段：本章定位 → 代码阅读
- 当前任务：阅读 `ch3/` 整体结构，从 `useEigen/eigenMatrix.cpp` 开始

## 各章节状态

### Chapter 1
状态：未记录
掌握：未知

### Chapter 2
状态：未记录
掌握：未知

### Chapter 3
状态：进行中（代码阅读）
掌握：

- Eigen 矩阵运算     🔴
- 旋转矩阵 / AngleAxis / 欧拉角 / 四元数转换  🔴
- Isometry3d / 刚体变换     🔴
- 坐标系变换 `T2w * T1w^{-1}`  🔴
- 轨迹位姿可视化     🔴

## 当前薄弱点

- 尚无历史记录。本章从基础开始，不假设已掌握 Eigen 几何模块。
- Eigen 是 header-only：`#include <Eigen/Core>` 本身正确，但编译器/clangd 必须带 `-I/usr/include/eigen3`。

## 高频错误

- 尚无。

## 错误记录

### 错误：`'Eigen/Core' file not found`

表现：
源文件 include 正确，但编辑器/直接 `g++ eigenMatrix.cpp` 报找不到头文件。

原因：
Debian/Ubuntu 把 Eigen 装在 `/usr/include/eigen3/Eigen/`，默认系统 include 路径只有 `/usr/include`。

正确理解：
需要把 `/usr/include/eigen3` 加入 include path。CMake 里用 `find_package(Eigen3)`，clangd 靠 `compile_commands.json` 或 `.clangd`。

再次检查：
遇到第三方头文件找不到时，先确认头文件实际路径，再确认编译命令是否带了对应 `-I`。

## 已完成项目

- 无

## 最近实验

- 无

## 下一步

- 阅读 `ch3/useEigen/eigenMatrix.cpp`，先自己判断矩阵类型、乘法与解方程在做什么。
