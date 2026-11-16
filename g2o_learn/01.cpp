#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>

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
            static_cast<const VertexX*>(_vertices[0]); // 基类 → 子类	恢复具体类型，访问对应接口

        _error[0] = _measurement - v->estimate();
    }

    // 读取观测数据
    bool read(std::istream& is) override {
        is >> _measurement;
        return static_cast<bool>(is);
    }

    // 写入观测数据
    bool write(std::ostream& os) const override {
        os << _measurement;
        return static_cast<bool>(os);
    }
};


// 第三部分：测试
int main() {

    // 创建顶点
    VertexX vertex;
    vertex.setEstimate(10.0);

    // 创建边
    EdgeX edge;

    // 将边连接到顶点
    edge.setVertex(0, &vertex); // 注意， 这里发生了指针类型的转换， 子类 → 基类	方便 g2o 统一管理顶点

    // 设置观测值
    edge.setMeasurement(12.0);

    // 计算误差
    edge.computeError();

    std::cout << "Estimate: "
              << vertex.estimate() << std::endl;

    std::cout << "Error: "
              << edge.error()[0] << std::endl;

    return 0;
}