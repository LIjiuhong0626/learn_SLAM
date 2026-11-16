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

    // 计算误差
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


    // 2. 创建一个 Vertex
    auto* vertex = new VertexX();

    vertex->setId(0);
    vertex->setEstimate(8.0);

    optimizer.addVertex(vertex);


    // 3. 创建三个 Edge

    Eigen::Matrix<double, 1, 1> information;
    information << 1.0;

    // Edge 1
    auto* edge1 = new EdgeX();

    edge1->setVertex(0, vertex);
    edge1->setMeasurement(10.0);
    edge1->setInformation(information);

    optimizer.addEdge(edge1);


    // Edge 2
    auto* edge2 = new EdgeX();

    edge2->setVertex(0, vertex);
    edge2->setMeasurement(12.0);
    edge2->setInformation(information);

    optimizer.addEdge(edge2);


    // Edge 3
    auto* edge3 = new EdgeX();

    edge3->setVertex(0, vertex);
    edge3->setMeasurement(14.0);
    edge3->setInformation(information);

    optimizer.addEdge(edge3);


    // 4. 初始化优化
    optimizer.initializeOptimization();

    // 5. 执行优化
    optimizer.optimize(10);


    // 6. 输出结果
    edge1->computeError();
    edge2->computeError();
    edge3->computeError();

    std::cout << "Final estimate: "
              << vertex->estimate() << std::endl;

    std::cout << "Edge 1 error: "
              << edge1->error()[0] << std::endl;

    std::cout << "Edge 2 error: "
              << edge2->error()[0] << std::endl;

    std::cout << "Edge 3 error: "
              << edge3->error()[0] << std::endl;

    return 0;
}