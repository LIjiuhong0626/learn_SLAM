#include <iostream>
#include <memory>
#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


// 第一部分：定义二维 Vertex
class VertexXY :
    public g2o::BaseVertex<2, Eigen::Vector2d> {

public:

    void setToOriginImpl() override {
        _estimate.setZero();
    }

    void oplusImpl(const double* update) override {

        _estimate[0] += update[0];
        _estimate[1] += update[1];

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


// 第二部分：定义绝对位置约束
class EdgeAbsoluteXY :
    public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexXY> {

public:

    // e = measurement - estimate
    void computeError() override {

        const VertexXY* v =
            static_cast<const VertexXY*>(_vertices[0]);

        _error = _measurement - v->estimate();

    }

    // J = -I
    void linearizeOplus() override {

        _jacobianOplusXi <<
            -1.0,  0.0,
             0.0, -1.0;

    }

    bool read(std::istream& is) override {

        is >> _measurement[0] >> _measurement[1];

        return static_cast<bool>(is);
    }

    bool write(std::ostream& os) const override {

        os << _measurement[0] << " "
           << _measurement[1];

        return static_cast<bool>(os);
    }
};


// 第三部分：定义相对位置约束
class EdgeRelativeXY :
    public g2o::BaseBinaryEdge<
        2, Eigen::Vector2d, VertexXY, VertexXY> {

public:

    // e = measurement - (x2 - x1)
    void computeError() override {

        const VertexXY* v1 =
            static_cast<const VertexXY*>(_vertices[0]);

        const VertexXY* v2 =
            static_cast<const VertexXY*>(_vertices[1]);

        _error =
            _measurement -
            (v2->estimate() - v1->estimate());

    }


    // J_i = I, J_j = -I
    void linearizeOplus() override {

        _jacobianOplusXi <<
             1.0, 0.0,
             0.0, 1.0;

        _jacobianOplusXj <<
            -1.0,  0.0,
             0.0, -1.0;

    }


    bool read(std::istream& is) override {

        is >> _measurement[0] >> _measurement[1];

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


    // 2. 创建两个独立的二维 Vertex
    auto* vertex1 = new VertexXY();
    auto* vertex2 = new VertexXY();

    vertex1->setId(0);
    vertex2->setId(1);

    vertex1->setEstimate(
        Eigen::Vector2d(0.0, 0.0)
    );

    vertex2->setEstimate(
        Eigen::Vector2d(0.0, 0.0)
    );

    optimizer.addVertex(vertex1);
    optimizer.addVertex(vertex2);


    // 3. 创建信息矩阵
    Eigen::Matrix2d information =
        Eigen::Matrix2d::Identity();


    // 4. 第一个 Unary Edge
    auto* edge1 = new EdgeAbsoluteXY();

    edge1->setVertex(0, vertex1);

    edge1->setMeasurement(
        Eigen::Vector2d(1.0, 2.0)
    );

    edge1->setInformation(information);

    optimizer.addEdge(edge1);


    // 5. 第二个 Unary Edge
    auto* edge2 = new EdgeAbsoluteXY();

    edge2->setVertex(0, vertex2);

    edge2->setMeasurement(
        Eigen::Vector2d(3.0, 4.0)
    );

    edge2->setInformation(information);

    optimizer.addEdge(edge2);


    // 6. Binary Edge：相对位置约束
    auto* edge12 = new EdgeRelativeXY();

    edge12->setVertex(0, vertex1);
    edge12->setVertex(1, vertex2);

    edge12->setMeasurement(
        Eigen::Vector2d(2.0, 2.0)
    );

    edge12->setInformation(information);

    optimizer.addEdge(edge12);


    // 7. 初始化优化
    optimizer.initializeOptimization();

    // 8. 执行优化
    optimizer.optimize(10);


    // 9. 输出结果
    edge1->computeError();
    edge2->computeError();
    edge12->computeError();

    std::cout << "Vertex 1:\n"
              << vertex1->estimate().transpose()
              << std::endl;

    std::cout << "Vertex 2:\n"
              << vertex2->estimate().transpose()
              << std::endl;

    std::cout << "Edge 1 error:\n"
              << edge1->error().transpose()
              << std::endl;

    std::cout << "Edge 2 error:\n"
              << edge2->error().transpose()
              << std::endl;

    std::cout << "Edge 12 error:\n"
              << edge12->error().transpose()
              << std::endl;

    return 0;
}