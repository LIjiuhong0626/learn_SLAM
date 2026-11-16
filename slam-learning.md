# SLAM Learning State

## 当前进度
- 当前章节：第 11、12 章；原第 3 章学习主线暂挂，第 2 章及以前按用户要求不学。
- 当前任务或阶段：已完成 `ch11`、`ch12` 的依赖、CMake、编译和自带数据运行验证。DBoW3 0.0.1 与 OctoMap 1.9.7 安装在 `~/.local`；两章共 7 个目标均已生成到各自 `build` 目录。
- 下一步：从 ch11 的 ORB 描述子、词袋字典、相似度评分与数据库查询开始读码；进入 ch12 时再分别学习单目深度滤波、RGB-D 点云、OctoMap 与 surfel 重建。

## 当前能力证据与薄弱点
- 🟡 用户能说出 `ldlt()` 返回分解对象，再由 `solve()` 利用分解解方程；但曾将 LDLT 说成 LU 后的“LULT”，分解的数学形式仍需在后续应用中验证。
- 未验证：用户能指出不熟悉的 Eigen/C++ 写法，但 `cast`、类模板构造、`clock()` 等尚无独立复述或改代码证据。
- 未验证：旋转矩阵作用在向量上的手算能力。
- 用户认为绕 z 轴 45° 旋转 \((1,0,0)\) 的手算“很简单”，但未给出结果，因此不作为已掌握证据；后续提高到复合变换。

## 值得复盘的错误、实验与项目
- 易混淆 LU、LLT 和 LDLT；后续遇到 SLAM 正规方程时再用实际矩阵区分。
- OpenCV 4 迁移：旧版全局宏应换成 `cv::IMREAD_*`、`cv::COLOR_*`、`cv::FM_8POINT`、`cv::FILLED` 等命名空间枚举；枚举语义不变，因此不会改变教学算法流程。
- CMake 子目录直接作为源码根目录配置时，不会继承 `ch5/CMakeLists.txt` 中的 `find_package` 结果；每个需要独立构建的示例必须自行查找 OpenCV 4 并传入 `${OpenCV_INCLUDE_DIRS}`。
- CMake 报“找不到 `CeresConfig.cmake`”时，先检查开发包和配置文件是否真实存在；只有 Ceres 已安装到非标准前缀时，设置 `Ceres_DIR`/`CMAKE_PREFIX_PATH` 才是正确解法。本机检查确认属于开发包未安装，而非路径遗漏。
- Ceres 2.0 的头文件使用 `std::integer_sequence`，本机以 C++11 编译会失败；第 9 章必须至少启用 C++14。该结论已由先失败、改为 C++14 后两个目标均构建成功验证。
- `g2o_viewer` 需要 Qt5、OpenGL 和 QGLViewer；本机原有 `/usr/local` g2o 核心库但没有 viewer，且无法无密码使用 `sudo`。最终用官方 `20241228_git` 发布版和用户级 QGLViewer 依赖构建隔离版本，安装到 `~/.local/opt/g2o-viewer-20241228`，通过 `~/.local/bin/g2o_viewer` 启动，避免覆盖现有 g2o。`ch10/sphere.g2o` 已在 WSLg 窗口中成功打开；命令行另验证载入 2500 个顶点、9799 条边，并完成一次优化迭代。
- ch11 原 CMake 把 DBoW3 静态库硬编码为 `/usr/local/lib/libDBoW3.a`；当前环境改为从 `~/.local` 自动发现共享库并以导入目标链接。教材常用的 DBoW3 提交 `c5ae539` 在 GCC 11 下缺少若干标准头，需要补入 `<cstddef>/<cstdint>/<istream>/<ostream>/<string>`；同时必须强制 C++11，不能只声明“至少 C++11”而让 GCC 默认使用 C++17，因为旧 API 含动态异常声明。
- Ubuntu 22.04 的 PCL 1.12 要求至少 C++14，且已移除 `MovingLeastSquares::setPolynomialFit()`；`setPolynomialOrder(order)` 在 `order > 1` 时已表示使用多项式拟合。ch12 因此固定 C++14，并使用当前 PCL API。OctoMap 1.9.7 以用户级开发包安装到 `~/.local`。
- ch12 自带 RGB-D 数据已验证：点云程序由 1,309,800 个点滤波到 31,876 个点；OctoMap 程序成功写出含 1,136,665 个节点的地图。单目程序仍需要仓库未附带的 REMODE 数据集，当前只验证了编译、链接和无参数用法提示。

## 待结问题链（仅用于续接与写笔记，不用于掌握评价）
- 问题链 ID：ch11-ch12-build-config-01
- 主题：配置并验证第 11、12 章环境
- 开始时间：2026-11-16
- 按顺序的关键问答、各轮章节与用户理解变化：
  1. 第 11、12 章。用户要求配置两个章节的环境。盘点确认 OpenCV 4.5.4、Eigen 3.4.0、PCL 1.12、Sophus 已存在，缺少 DBoW3 与 OctoMap 开发库；原 CMake 还包含 DBoW3 绝对路径和旧式全局编译选项。
  2. 第 11 章。采用与教材匹配的 DBoW3 提交 `c5ae539`，修复 GCC 11 对缺失标准头的报错后构建，并安装到 `~/.local`。重写 ch11 CMake，使其自动发现 DBoW3/OpenCV、使用导入目标并强制 C++11。三个目标均编译成功；`feature_training` 成功生成 4972 词字典，`loop_closure` 成功完成十张图的相似度和数据库查询。
  3. 第 12 章。将 Ubuntu 22.04 的 OctoMap 1.9.7 开发包和运行库安装到 `~/.local`；CMake 改为使用 Eigen3、Sophus、PCL 和 OctoMap 目标，并因 PCL 1.12 要求固定 C++14。移除已废弃的 `setPolynomialFit()`，保留等价的 `setPolynomialOrder()`；为 `dense_mapping::update()` 补上成功返回值。
  4. 第 12 章。四个目标均编译成功，动态库无缺失；自带 RGB-D 数据运行得到 31,876 点的滤波地图，并写出 1,136,665 节点的 OctoMap。单目程序需要外部 REMODE 数据集，因此当前只完成编译、链接与参数检查。用户对这些依赖/API 变化和两章算法的理解尚未验证。
  5. 第 11 章。用户反馈 VS Code 显示 `cannot open source file "DBoW3/DBoW3.h"`。核对确认头文件存在于 `~/.local/include/DBoW3/DBoW3.h`，CMake 构建也已成功，问题属于 IntelliSense 的 `includePath` 漏配而非依赖缺失；已在 `.vscode/c_cpp_properties.json` 中加入 `${env:HOME}/.local/include`，并用相同包含路径执行 `g++ -fsyntax-only` 验证通过。用户是否已看到编辑器诊断消失尚待确认。

## 已结待归档队列（仅用于写笔记，不用于掌握评价）
- 无
