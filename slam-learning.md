# SLAM Learning State

## 当前进度
- 当前章节：第 3 章 三维空间刚体运动（`ch3/`）。第 2 章及以前按用户要求不学。
- 当前任务或阶段：已结束 `eigenMatrix.cpp` 问题链，进入 `ch3/useGeometry/eigenGeometry.cpp`，从旋转向量 `AngleAxisd` 与旋转矩阵开始。
- 下一步：用户先预测绕 z 轴正向旋转 45° 后 \((1,0,0)\) 的坐标，再对照 `rotation_vector * v` 和 `rotation_matrix * v`。

## 当前能力证据与薄弱点
- 🟡 用户能说出 `ldlt()` 返回分解对象，再由 `solve()` 利用分解解方程；但曾将 LDLT 说成 LU 后的“LULT”，分解的数学形式仍需在后续应用中验证。
- 未验证：用户能指出不熟悉的 Eigen/C++ 写法，但 `cast`、类模板构造、`clock()` 等尚无独立复述或改代码证据。
- 未验证：旋转矩阵作用在向量上的手算能力。

## 值得复盘的错误、实验与项目
- 易混淆 LU、LLT 和 LDLT；后续遇到 SLAM 正规方程时再用实际矩阵区分。

## 待结问题链（仅用于续接与写笔记，不用于掌握评价）
- 问题链 ID：ch3-03
- 主题：`eigenGeometry.cpp` 的旋转表示与向量旋转
- 开始时间：2026-10-01
- 按顺序的关键问答、各轮章节与用户理解变化：
  1. 第 3 章。在用户通过分解对象的断点检查后，转入 `eigenGeometry.cpp`。回答先对 LDLT 做最后纠正，再将 `AngleAxisd(M_PI/4, Vector3d(0,0,1))` 映射为绕 z 轴正向旋转 45°，要求用户在运行代码前预测 `rotation_vector * Vector3d(1,0,0)` 的结果。用户尚未回答。

## 已结待归档队列（仅用于写笔记，不用于掌握评价）
- 无
