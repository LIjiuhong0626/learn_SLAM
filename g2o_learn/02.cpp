#include <iostream>
#include <memory>
#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


// 第一部分：定义顶点
class VertexX : public g2o::BaseVertex<1, double> {
public:

    void setToOriginImpl() override {
        _estimate = 0.0;
    }

    void oplusImpl(const double* update) override {
        _estimate += update[0];
    }

    bool read(std::istream& is) override {
        is >> _estimate;
        return static_cast<bool>(is);
    }

    bool write(std::ostream& os) const override {
        os << _estimate;
        return static_cast<bool>(os);
    }
};


// 第二部分：定义一元边
class EdgeX : public g2o::BaseUnaryEdge<1, double, VertexX> {
public:

    void computeError() override {

        const VertexX* v =
            static_cast<const VertexX*>(_vertices[0]);

        _error[0] = _measurement - v->estimate();
    }

    bool read(std::istream& is) override {
        is >> _measurement;
        return static_cast<bool>(is);
    }

    bool write(std::ostream& os) const override {
        os << _measurement;
        return static_cast<bool>(os);
    }
};


// 第三部分：真正执行优化
int main() {

    // 1. 设置求解器类型
    using BlockSolverType =
        g2o::BlockSolver<g2o::BlockSolverTraits<1, 1>>;

    using LinearSolverType =
        g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

    // 2. 创建优化器
    g2o::SparseOptimizer optimizer; // 统一管理的， 里面有顶点， 边， 以及算法

    // 3. 设置优化算法
    auto* algorithm = new g2o::OptimizationAlgorithmGaussNewton( // 这里说明我们用 高斯牛顿法 迭代
        std::make_unique<BlockSolverType>(// 这里是 块，如果是在 稀疏性 问题下我们就要对 H 矩阵分块进行， 这里的 make_unique 是 一个对象在同一时间只能有一个所有者
            std::make_unique<LinearSolverType>() // 用什么方法 求解 H Δx = -b
        )
    );

    optimizer.setAlgorithm(algorithm);

    // 打开优化过程的输出
    optimizer.setVerbose(true);


    // 4. 创建顶点并加入优化器
    auto* vertex = new VertexX();

    vertex->setId(0);
    vertex->setEstimate(10.0);

    optimizer.addVertex(vertex);


    // 5. 创建边并加入优化器
    auto* edge = new EdgeX();

    edge->setVertex(0, vertex);
    edge->setMeasurement(12.0);

    edge->setInformation(
        Eigen::Matrix<double, 1, 1>::Identity() // 设置信息矩阵 这里是 1 维度的
    );

    optimizer.addEdge(edge);


    // 6. 初始化优化
    optimizer.initializeOptimization();

    // 7. 执行优化，最多迭代10次
    optimizer.optimize(10);


    // 8. 输出优化结果
    edge->computeError();

    std::cout << "Final estimate: "
              << vertex->estimate() << std::endl;

    std::cout << "Final error: "
              << edge->error()[0] << std::endl;

    return 0;
}