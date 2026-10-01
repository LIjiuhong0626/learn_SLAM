# 第 3 章 三维空间刚体运动

## 2026-09-28｜ch3-01｜主题：旋转矩阵与旋转向量如何作用在向量上

- 所属章节与情境：第 3 章起步。最初从 `ch3/useGeometry/eigenGeometry.cpp` 进入旋转；用户要求改为先讲 `ch3/useEigen/eigenMatrix.cpp`，本链在此结束。
- 提问与回答链：
  1. 用户要求跳过此前章节，建好记录并开始第 3 章。当时尚未展示对代码的理解。
     回答从 `AngleAxisd(π/4, (0,0,1))` 入手：`toRotationMatrix()` 对应绕 z 轴的旋转矩阵，`rotation_vector * v` 与 `rotation_matrix * v` 都计算 \(Rv\)。对 \(v=(1,0,0)\)，\(R_z(\pi/4)v=(\cos\pi/4,\sin\pi/4,0)\)。源码注释写“坐标变换”，该乘法是在固定坐标系里旋转向量。已请用户手算并说明几何意义。
  2. 用户未作答，要求讲解起点改为 `eigenMatrix.cpp`。
     旋转部分因此暂停，留到几何模块再继续。
- 理解变化：用户未回答手算题。旋转矩阵作用在向量上的理解尚未验证。
- 最终结论：绕 z 轴 \(\theta\) 的旋转矩阵为 \(R_z(\theta)\)，左乘向量得到固定坐标系下的新方向；\((1,0,0)\) 绕 z 转 \(\pi/4\) 得到 \((\cos\pi/4,\sin\pi/4,0)\)。此结论来自讲解，未经用户复述或手算确认。
- 待验证或后续练习：回到 `eigenGeometry.cpp` 时，让用户手算该结果并说明几何意义。

## 2026-10-01｜问题链 ch3-02｜主题：`eigenMatrix.cpp` 的矩阵类型与分解求解

- 所属章节与情境：第 3 章，阅读 `ch3/useEigen/eigenMatrix.cpp`，建立 Eigen 类型、表达式和线性方程分解对象的基础。
- 提问与回答链：
  1. 用户要求从该文件开始讲解。回答依次说明 `Matrix<标量,行,列>`、逗号初始化、矩阵乘法的类型/维度约束、对称特征分解，以及求解同一个 \(Ax=b\) 的求逆、QR 和 LDLT 三种写法。
  2. 用户追问 `Matrix3d::Random()`、`.cast<double>()`、`SelfAdjointEigenSolver`、`clock()` 和 Cholesky。回答说明 `::` 是作用域解析，`cast` 做逐元素数值转换，`SelfAdjointEigenSolver` 是针对自伴矩阵的类模板，`clock()` 测量进程 CPU 时间；并区分经典 \(A=LL^T\) 与代码里带主元的 LDLT。
  3. 用户要求从源码解释“成员函数模板”，又追问 `<double>` 后的 `()` 及 `static_cast`。回答沿 Eigen 继承与头文件展开路径，将 `cast<double>()` 对应到成员函数模板的模板实参和无参调用，最后落到单个元素的 `static_cast<double>(x)`。
  4. 用户单独追问 Cholesky。回答用 2×2 数值例子走完 \(A=LL^T\)，再把 \(Ax=b\) 拆成两次三角回代；说明 `llt()` 对应经典 Cholesky，而示例使用 `ldlt()`。
  5. 用户要求比较 Eigen 与 MATLAB，并追问 MATLAB 的性能及 `solve` 外的用法。回答对齐矩阵构造、乘法、转置、特征值和线性求解语法，并区分 MATLAB 在大矩阵 BLAS/LAPACK 上的优势与 Eigen 固定小矩阵在 SLAM 循环中的优势；说明 LDLT 对象还可重复求解、检查分解结果或做秩更新。
  6. 用户问为何特征分解直接构造 `SelfAdjointEigenSolver`，而 LDLT 可写成成员函数。回答说明 `A.ldlt()` 只是包装 `LDLT<...>(A)` 的便捷成员函数，两者都会得到保存分解结果的对象；“短函数”并非 C++ 术语。
  7. 断点检查时，用户回答 `ldlt()` 会返回类似分解结果的东西，再用它解方程；但认为正定矩阵做 LU 后“变成了 LULT”。回答肯定其对“分解对象→求解”的理解，并纠正 LDLT 不是 LU 的变形。
- 理解变化：用户已能说出 `ldlt()` 返回分解对象、`solve()` 利用它解方程；但将 LDLT 与 LU 混淆，对分解形式的准确表达仍需验证。
- 最终结论：`matrix_NN.ldlt()` 构造 LDLT 分解对象，对称情形可写为带置换的 \(A=P^TLDL^TP\)（实数情形）；`.solve(v_Nd)` 用该分解求 \(Ax=b\)，不必显式计算 \(A^{-1}\)。
- 待验证或后续练习：在后续遇到正定规方程时，让用户再次区分 LU、LLT 和 LDLT，并说明 \(MM^T\) 为何保证半正定而不是无条件正定。
