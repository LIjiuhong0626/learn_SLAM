# 第 9 章学习问答笔记

## 2026-11-16｜ch09-build-config-01｜主题：配置并编译第 9 章 BA 示例

- 所属章节与情境：第 9 章后端 BA；目标是让 Ceres、g2o 示例和 `g2o_viewer` 在当前 Ubuntu/WSLg 环境可编译、可运行。
- 提问与回答链：
  1. 用户要求配置 ch9。检查确认系统已有 Eigen、CSparse 和 `/usr/local` 下的 g2o，但缺少 Ceres 开发包；同时保留了用户在 `bundle_adjustment_g2o.cpp` 中的既有修改。配置过程先修正 CMake 最低版本及 `FindCSparse.cmake` 的包名警告，再实际编译定位错误。
  2. 首次编译表明 Ceres 2.0 的头文件使用 `std::integer_sequence`，C++11 不足；将第 9 章标准提高到 C++14 后，`bundle_adjustment_ceres` 与 `bundle_adjustment_g2o` 都成功生成。沙箱中的 ccache 缓存目录只读，验证时以 `CCACHE_DISABLE=1` 绕过，这不是源码问题。
  3. 用户继续要求配置 `g2o_viewer`。系统的 g2o 安装没有 viewer 和 QGLViewer，且无法免密 sudo；因此采用隔离的用户级安装，使用 g2o 官方 `20241228_git` 和用户级 QGLViewer 2.6.3 构建 viewer、类型与求解器插件，安装到 `~/.local/opt/g2o-viewer-20241228`，没有覆盖原有 g2o。
  4. 最后验证了动态库、类型注册及 CSparse/CHOLMOD/Eigen 求解器；`ch10/sphere.g2o` 能在 WSLg 窗口中打开，命令行载入 2500 个顶点、9799 条边并完成一次 Gauss–Newton 迭代。
- 理解变化：本链以工程配置和复现为主；用户对 C++14 原因、viewer 操作与图结构的理解尚未验证。
- 最终结论：第 9 章两个 BA 目标和隔离的 `g2o_viewer` 环境均已可用；依赖缺失、语言标准和用户级安装路径已由实际构建与运行验证。
- 待验证或后续练习：分别运行 Ceres/g2o BA，比较代价下降、收敛速度和点云结果；随后阅读 BA 图结构与 Schur 消元实现。
