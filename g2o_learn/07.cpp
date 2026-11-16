#include <iostream>
#include <memory>
#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


// 第一部分：定义二维顶点
class VertexXY :
    public g2o::BaseVertex<2, Eigen::Vector2d> { // 2 维度， 传入的估计值为 Eigen::Vector2d 类型

public:

    // 设置初始原点
    void setToOriginImpl() override {
        _estimate.setZero(); // 这里的 _estimate 就是 v2d 类型的， setzero 函数就是 eigen 的函数
    }

    // 二维增量更新
    void oplusImpl(const double* update) override { // 这里传入的都是 都崩了 

        _estimate[0] += update[0];
        _estimate[1] += update[1]; // 两个分量都要更新

    }

    bool read(std::istream& is) override {

        is >> _estimate[0] >> _estimate[1];

        return static_cast<bool>(is);
    }

    bool write(std::ostream& os) const override {

        os << _estimate[0] << " "
           << _estimate[1];

        return static_cast<bool>(os);
    }
};


// 第二部分：定义二维残差边
class EdgeXY :
    public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexXY> { // 残差维度为2， 观测值类型是 Eigen::Vector2d， 链接顶点类型为 。。。 

public:

    // 计算二维残差
    void computeError() override {

        const VertexXY* v =
            static_cast<const VertexXY*>(_vertices[0]);

        const Eigen::Vector2d& estimate = v->estimate();

        Eigen::Vector2d prediction;

        prediction[0] =
            estimate[0] + 2.0 * estimate[1];

        prediction[1] =
            3.0 * estimate[0] - estimate[1];

        _error = _measurement - prediction; // error 直接就是 v2d 类型哇

    }


    // 手动计算 2×2 Jacobian
    void linearizeOplus() override {

        _jacobianOplusXi(0, 0) = -1.0;
        _jacobianOplusXi(0, 1) = -2.0;

        _jacobianOplusXi(1, 0) = -3.0;
        _jacobianOplusXi(1, 1) = 1.0;

    }


    bool read(std::istream& is) override {

        is >> _measurement[0]
           >> _measurement[1];

        return static_cast<bool>(is);
    }

    bool write(std::ostream& os) const override {

        os << _measurement[0] << " "
           << _measurement[1];

        return static_cast<bool>(os);
    }
};


int main() {

    // 1. 配置求解器
    using BlockSolverType =
        g2o::BlockSolver<g2o::BlockSolverTraits<2, 2>>;

    using LinearSolverType =
        g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

    g2o::SparseOptimizer optimizer;

    auto* algorithm =
        new g2o::OptimizationAlgorithmGaussNewton(
            std::make_unique<BlockSolverType>(
                std::make_unique<LinearSolverType>()
            )
        );

    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(true);


    // 2. 创建二维 Vertex
    auto* vertex = new VertexXY();

    vertex->setId(0);

    vertex->setEstimate(
        Eigen::Vector2d(0.0, 0.0)
    );

    optimizer.addVertex(vertex);


    // 3. 创建二维 Edge
    auto* edge = new EdgeXY();

    edge->setVertex(0, vertex);

    edge->setMeasurement(
        Eigen::Vector2d(4.0, 5.0)
    );

    edge->setInformation(
        Eigen::Matrix2d::Identity()
    );

    optimizer.addEdge(edge);


    // 4. 初始化优化
    optimizer.initializeOptimization();

    // 5. 执行优化
    optimizer.optimize(10);


    // 6. 输出结果
    edge->computeError();

    std::cout << "Final estimate:\n"
              << vertex->estimate() << std::endl;

    std::cout << "Final error:\n"
              << edge->error() << std::endl;

    std::cout << "Final chi2:\n"
              << edge->chi2() << std::endl;

    return 0;
}