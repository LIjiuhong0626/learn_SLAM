
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


// ============================================================
// 第一部分：定义顶点
// ============================================================

class VertexX : public g2o::BaseVertex<1, double> {

public:

    // 将顶点估计值初始化为 0
    void setToOriginImpl() override {
        _estimate = 0.0;
    }

    // 根据优化器给出的增量更新估计值
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


// ============================================================
// 第二部分：定义一元边
// ============================================================

class EdgeX : public g2o::BaseUnaryEdge<1, double, VertexX> {

public:

    // 计算一元边的残差
    // error = measurement - estimate
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


// ============================================================
// 第三部分：定义二元边
// ============================================================

class EdgeDifference :
    public g2o::BaseBinaryEdge<1, double, VertexX, VertexX> {

public:

    // 计算两个顶点之间的相对误差
    // error = measurement - (x - y)
    void computeError() override {

        const VertexX* v1 =
            static_cast<const VertexX*>(_vertices[0]);

        const VertexX* v2 =
            static_cast<const VertexX*>(_vertices[1]);

        _error[0] =
            _measurement - (v1->estimate() - v2->estimate());
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


// ============================================================
// 第四部分：主函数
// ============================================================

int main() {

    // --------------------------------------------------------
    // 1. 配置求解器
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // 2. 创建两个顶点
    // --------------------------------------------------------

    auto* vertexX = new VertexX();
    auto* vertexY = new VertexX();

    vertexX->setId(0);
    vertexY->setId(1);

    vertexX->setEstimate(0.0);
    vertexY->setEstimate(0.0);

    optimizer.addVertex(vertexX);
    optimizer.addVertex(vertexY);


    // --------------------------------------------------------
    // 3. 创建信息矩阵
    // --------------------------------------------------------

    Eigen::Matrix<double, 1, 1> information;
    information << 1.0;


    // --------------------------------------------------------
    // 4. 创建一元边：约束 x 接近 2
    // --------------------------------------------------------

    auto* edgeX = new EdgeX();

    edgeX->setVertex(0, vertexX);
    edgeX->setMeasurement(2.0);
    edgeX->setInformation(information);

    optimizer.addEdge(edgeX);


    // --------------------------------------------------------
    // 5. 创建三个一元边：约束 y 分别接近 1、3、10
    // --------------------------------------------------------

    const double yMeasurements[] = {1.0, 3.0, 10.0};

    std::vector<EdgeX*> yEdges;

    for (double measurement : yMeasurements) {

        auto* edge = new EdgeX();

        edge->setVertex(0, vertexY);
        edge->setMeasurement(measurement);
        edge->setInformation(information);

        optimizer.addEdge(edge);

        yEdges.push_back(edge);
    }


    // --------------------------------------------------------
    // 6. 创建二元边：约束 x - y 接近 1
    // --------------------------------------------------------

    auto* edgeXY = new EdgeDifference();

    edgeXY->setVertex(0, vertexX);
    edgeXY->setVertex(1, vertexY);

    edgeXY->setMeasurement(1.0);
    edgeXY->setInformation(information);

    optimizer.addEdge(edgeXY);


    // --------------------------------------------------------
    // 7. 初始化优化
    // --------------------------------------------------------

    if (!optimizer.initializeOptimization()) {

        std::cerr << "Optimization initialization failed!"
                  << std::endl;

        return -1;
    }


    // --------------------------------------------------------
    // 8. 执行优化
    // --------------------------------------------------------

    int iterations = optimizer.optimize(10);

    std::cout << "\nIterations: " << iterations << std::endl;


    // --------------------------------------------------------
    // 9. 输出顶点估计值
    // --------------------------------------------------------

    std::cout << "\n========== Final Estimates ==========" << std::endl;

    std::cout << "Final x: "
              << vertexX->estimate() << std::endl;

    std::cout << "Final y: "
              << vertexY->estimate() << std::endl;


    // --------------------------------------------------------
    // 10. 重新计算所有边的误差
    // --------------------------------------------------------

    edgeX->computeError();

    for (auto* edge : yEdges) {
        edge->computeError();
    }

    edgeXY->computeError();


    // --------------------------------------------------------
    // 11. 输出所有边的误差
    // --------------------------------------------------------

    std::cout << "\n========== Final Errors ==========" << std::endl;

    std::cout << "Edge X error: "
              << edgeX->error()[0] << std::endl;

    for (size_t i = 0; i < yEdges.size(); ++i) {

        std::cout << "Edge Y[" << i << "] error: "
                  << yEdges[i]->error()[0] << std::endl;
    }

    std::cout << "Edge XY error: "
              << edgeXY->error()[0] << std::endl;


    return 0;
}
