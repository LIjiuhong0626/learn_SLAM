# SLAM Learning State

## 当前进度
- 当前章节：跨章节工程维护（`ch5/`—`ch8/`、`ch11/`—`ch13/`）；原第 3 章学习主线暂挂，第 2 章及以前按用户要求不学。
- 当前任务或阶段：OpenCV 4 迁移后的构建验证；已补齐第 5 章三个示例子目录的独立 CMake 配置，使其既能单独运行 `cmake ..`，也能由 `ch5/` 顶层统一构建。
- 下一步：用户在 `ch5/imageBasics/build` 重新运行 `cb` 验证本地命令；确认后可提交修复，再继续第 3 章四元数主线。

## 当前能力证据与薄弱点
- 🟡 用户能说出 `ldlt()` 返回分解对象，再由 `solve()` 利用分解解方程；但曾将 LDLT 说成 LU 后的“LULT”，分解的数学形式仍需在后续应用中验证。
- 未验证：用户能指出不熟悉的 Eigen/C++ 写法，但 `cast`、类模板构造、`clock()` 等尚无独立复述或改代码证据。
- 未验证：旋转矩阵作用在向量上的手算能力。
- 用户认为绕 z 轴 45° 旋转 \((1,0,0)\) 的手算“很简单”，但未给出结果，因此不作为已掌握证据；后续提高到复合变换。

## 值得复盘的错误、实验与项目
- 易混淆 LU、LLT 和 LDLT；后续遇到 SLAM 正规方程时再用实际矩阵区分。
- OpenCV 4 迁移：旧版全局宏应换成 `cv::IMREAD_*`、`cv::COLOR_*`、`cv::FM_8POINT`、`cv::FILLED` 等命名空间枚举；枚举语义不变，因此不会改变教学算法流程。
- CMake 子目录直接作为源码根目录配置时，不会继承 `ch5/CMakeLists.txt` 中的 `find_package` 结果；每个需要独立构建的示例必须自行查找 OpenCV 4 并传入 `${OpenCV_INCLUDE_DIRS}`。

## 待结问题链（仅用于续接与写笔记，不用于掌握评价）
- 问题链 ID：project-opencv4-01
- 主题：全仓库只保留 OpenCV 4，并迁移教学代码
- 开始时间：2026-10-15
- 按顺序的关键问答、各轮章节与用户理解变化：
  1. 涉及第 5—8、11—13 章。用户要求删除 OpenCV 3、只保留 OpenCV 4，并同步修改教学代码、添加注释但不改变源码逻辑。
  2. 审计确认系统安装为 OpenCV 4.5.4，另有独立的用户级 OpenCV 3.4.20；仓库多章 CMake 仍要求 OpenCV 3，源码还使用旧头文件路径和全局常量。回答据此把 CMake 最低版本统一为 4，并将旧 API 写法映射为 OpenCV 4 的命名空间枚举，保留所有参数和控制流。
  3. 已删除用户级 OpenCV 3.4.20 目录及第 5 章对应的 `build/opencv3` 构建产物。第 5、8、12 章相关目标完成 OpenCV 4 构建；第 7 章的 OpenCV-only 目标完成构建。其余章节均已在配置阶段确认找到 OpenCV 4.5.4，后续构建分别受 Ceres/g2o/DBoW3/OctoMap/glog 缺失影响，与本次 OpenCV 迁移无关。
  4. 用户在 `ch5/imageBasics/build` 用 `cb` 独立构建时报 `opencv2/*.hpp` 不存在。检查生成的 `flags.make` 确认包含路径为空：该子目录的 CMake 依赖顶层 `ch5` 传入 OpenCV 变量，单独配置时没有执行 `find_package`。修复后编译命令含 `-isystem /usr/include/opencv4`，`imageBasics` 与 `undistortImage` 均构建通过；进一步审计并修复相同的 `rgbd` 独立构建问题，`stereo`、`rgbd` 及第 5 章顶层全量构建也全部通过。

## 已结待归档队列（仅用于写笔记，不用于掌握评价）
- 无
