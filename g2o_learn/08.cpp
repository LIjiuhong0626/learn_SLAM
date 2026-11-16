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


// 第二部分：定义约束 x = 2
class EdgeX :
    public g2o::BaseUnaryEdge<1, double, VertexXY> {

public:

    void computeError() override {

        const VertexXY* v =
            static_cast<const VertexXY*>(_vertices[0]);

        _error[0] = _measurement - v->estimate()[0];
    }

    void linearizeOplus() override {

        _jacobianOplusXi(0, 0) = -1.0;
        _jacobianOplusXi(0, 1) = 0.0;

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


// 第三部分：定义约束 y = 1
class EdgeY :
    public g2o::BaseUnaryEdge<1, double, VertexXY> {

public:

    void computeError() override {

        const VertexXY* v =
            static_cast<const VertexXY*>(_vertices[0]);

        _error[0] = _measurement - v->estimate()[1];
    }

    void linearizeOplus() override {

        _jacobianOplusXi(0, 0) = 0.0;
        _jacobianOplusXi(0, 1) = -1.0;

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


// 第四部分：定义约束 x + y = 3
class EdgeSum :
    public g2o::BaseUnaryEdge<1, double, VertexXY> {

public:

    void computeError() override {

        const VertexXY* v =
            static_cast<const VertexXY*>(_vertices[0]);

        _error[0] =
            _measurement -
            (v->estimate()[0] + v->estimate()[1]);

    }

    void linearizeOplus() override {

        _jacobianOplusXi(0, 0) = -1.0;
        _jacobianOplusXi(0, 1) = -1.0;

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


    // 3. 创建信息矩阵
    Eigen::Matrix<double, 1, 1> information;
    information << 1.0;


    // 4. 创建 Edge 1：x = 2
    auto* edgeX = new EdgeX();

    edgeX->setVertex(0, vertex);
    edgeX->setMeasurement(2.0);
    edgeX->setInformation(information);

    optimizer.addEdge(edgeX);


    // 5. 创建 Edge 2：y = 1
    auto* edgeY = new EdgeY();

    edgeY->setVertex(0, vertex);
    edgeY->setMeasurement(1.0);
    edgeY->setInformation(information);

    optimizer.addEdge(edgeY);


    // 6. 创建 Edge 3：x + y = 3
    auto* edgeSum = new EdgeSum();

    edgeSum->setVertex(0, vertex);
    edgeSum->setMeasurement(3.0);
    edgeSum->setInformation(information);

    optimizer.addEdge(edgeSum);


    // 7. 初始化优化
    optimizer.initializeOptimization();

    // 8. 执行优化
    optimizer.optimize(10);


    // 9. 输出结果
    edgeX->computeError();
    edgeY->computeError();
    edgeSum->computeError();

    std::cout << "Final estimate:\n"
              << vertex->estimate() << std::endl;

    std::cout << "Edge X error: "
              << edgeX->error()[0] << std::endl;

    std::cout << "Edge Y error: "
              << edgeY->error()[0] << std::endl;

    std::cout << "Edge Sum error: "
              << edgeSum->error()[0] << std::endl;

    std::cout << "Total chi2: "
              << optimizer.activeChi2() << std::endl;

    return 0;
}