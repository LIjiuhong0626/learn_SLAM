#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <cmath>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
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

        _error[0] = _measurement - std::exp(v->estimate()) - v->estimate() * v->estimate();
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


// 第三部分：定义二元边
class EdgeDifference :
    public g2o::BaseBinaryEdge<1, double, VertexX, VertexX> {

public:

    void computeError() override {

        const VertexX* v1 =
            static_cast<const VertexX*>(_vertices[0]);

        const VertexX* v2 =
            static_cast<const VertexX*>(_vertices[1]);

        _error[0] =
            _measurement - (v1->estimate() - v2->estimate());
    }


    // 手动计算解析 Jacobian
    // void linearizeOplus() override {

    //     _jacobianOplusXi(0, 0) = -1.0; // 把雅可比矩阵第 0 行、第 0 列的元素设置为 -1.0 ，这里是对 x 的雅克比矩阵

    //     _jacobianOplusXj(0, 0) = 1.0;
    // }


    bool read(std::istream& is) override {
        is >> _measurement;
        return static_cast<bool>(is);
    }

    bool write(std::ostream& os) const override {
        os << _measurement;
        return static_cast<bool>(os);
    }
};


int main() {

    // 1. 配置求解器
    using BlockSolverType =
        g2o::BlockSolver<g2o::BlockSolverTraits<1, 1>>;

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


    // 2. 创建两个顶点
    auto* vertexX = new VertexX();
    auto* vertexY = new VertexX();

    vertexX->setId(0);
    vertexY->setId(1);

    vertexX->setEstimate(0.0);
    vertexY->setEstimate(0.0);

    optimizer.addVertex(vertexX);
    optimizer.addVertex(vertexY);


    // 3. 信息矩阵
    Eigen::Matrix<double, 1, 1> information;
    information << 1.0;


    // 4. Unary Edge X
    auto* edgeX = new EdgeX();

    edgeX->setVertex(0, vertexX);
    edgeX->setMeasurement(2.0);
    edgeX->setInformation(information);

    optimizer.addEdge(edgeX);


    // 5. Unary Edge Y
    auto* edgeY = new EdgeX();

    edgeY->setVertex(0, vertexY);
    edgeY->setMeasurement(1.0);
    edgeY->setInformation(information);

    optimizer.addEdge(edgeY);


    // 6. Binary Edge
    auto* edgeXY = new EdgeDifference();

    edgeXY->setVertex(0, vertexX);
    edgeXY->setVertex(1, vertexY);

    edgeXY->setMeasurement(1.0);
    edgeXY->setInformation(information);

    optimizer.addEdge(edgeXY);


    // 7. 初始化并执行优化
    optimizer.initializeOptimization();
    optimizer.optimize(10);


    // 8. 输出结果
    edgeX->computeError();
    edgeY->computeError();
    edgeXY->computeError();

    std::cout << "Final x: "
              << vertexX->estimate() << std::endl;

    std::cout << "Final y: "
              << vertexY->estimate() << std::endl;

    std::cout << "Edge X error: "
              << edgeX->error()[0] << std::endl;

    std::cout << "Edge Y error: "
              << edgeY->error()[0] << std::endl;

    std::cout << "Edge XY error: "
              << edgeXY->error()[0] << std::endl;

    return 0;
}