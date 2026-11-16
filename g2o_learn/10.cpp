#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>

// 位姿 T cw
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{ // 优化变量的局部更新维度， 我们之后 update 也是直接这样优化； 这个点的类型是 SE3d， 也就是一个 T

public:
    void setToOriginImpl() override
    {

        _estimate = Sophus::SE3d(); // 初始化
    }

    void oplusImpl(const double *update) override
    { // 6 就是在这里用的， 这里是优化器通过高斯牛顿求解出了一个 xi， 因为我们的近似方程就是用的 delta xi 啊，所以这里得到的就是 xi

        Eigen::Matrix<double, 6, 1> delta;

        delta = Eigen::Map<
            const Eigen::Matrix<double, 6, 1> // 把一个普通的 double 指针，映射成 Eigen 能够识别的六维向量
            >(update);                        // 这里的 map 是指将一个指针类型映射成 map 里面的类型， 之后我可以用 delta 代替这个 update，
                                              // 但是操作是用 这个类型进行操作， 此外 map 没有额外开辟空间, 更改的是 数组中的值， 但是 const，意味着映射得到的矩阵是只读的

        // 左扰动
        _estimate =
            Sophus::SE3d::exp(delta) * _estimate; // 将
    }

    bool read(std::istream &is) override
    {

        Eigen::Matrix<double, 6, 1> data;

        for (int i = 0; i < 6; i++)
        {
            is >> data[i];
        }

        if (!is)
            return false;

        _estimate = Sophus::SE3d::exp(data); // 存的都是 T， 也就是 李代数 exp 之后的结果！

        return true;
    }

    bool write(std::ostream &os) const override
    {

        Eigen::Matrix<double, 6, 1> data =
            _estimate.log();

        os << data.transpose();

        return static_cast<bool>(os);
    }
};

// 三维 点
class VertexPoint : public g2o::BaseVertex<3, Eigen::Vector3d>
{

public:
    void setToOriginImpl() override
    {

        _estimate.setZero();
    }

    void oplusImpl(const double *update) override
    {

        _estimate += Eigen::Map<
            const Eigen::Vector3d>(update);
    }

    bool read(std::istream &is) override
    {

        is >> _estimate[0] >> _estimate[1] >> _estimate[2];

        return static_cast<bool>(is);
    }

    bool write(std::ostream &os) const override
    {

        os << _estimate.transpose();

        return static_cast<bool>(os);
    }
};

class EdgeProjection : public g2o::BaseBinaryEdge<
                           2,
                           Eigen::Vector2d,
                           VertexPose,
                           VertexPoint>
{

public:
    EdgeProjection(
        double fx,
        double fy,
        double cx,
        double cy)
        : _fx(fx),
          _fy(fy),
          _cx(cx),
          _cy(cy) {}

    // 第一部分：计算重投影误差
    void computeError() override
    {

        const VertexPose *pose =
            static_cast<const VertexPose *>(
                _vertices[0]);

        const VertexPoint *point =
            static_cast<const VertexPoint *>(
                _vertices[1]);

        // 世界坐标系 -> 相机坐标系
        Eigen::Vector3d pc =
            pose->estimate() * point->estimate(); // 本来维度有问题的，但是这里帮我们重载了， 变成三维的乘法

        double x = pc[0];
        double y = pc[1];
        double z = pc[2];

        // 针孔相机投影
        double u = _fx * x / z + _cx;
        double v = _fy * y / z + _cy;

        // 观测值 - 预测值
        _error[0] = _measurement[0] - u;
        _error[1] = _measurement[1] - v;
    }

    // 第二部分：计算解析 Jacobian
    void linearizeOplus() override
    {

        const VertexPose *pose =
            static_cast<const VertexPose *>(
                _vertices[0]);

        const VertexPoint *point =
            static_cast<const VertexPoint *>(
                _vertices[1]);

        const Sophus::SE3d &Tcw = // const：不允许修改 Tcw
            pose->estimate(); // &：引用，避免复制整个 SE(3) 对象

        Eigen::Vector3d pc =
            Tcw * point->estimate();

        double x = pc[0];
        double y = pc[1];
        double z = pc[2];

        // 1. 投影 Jacobian：2 x 3
        Eigen::Matrix<double, 2, 3> J_proj;

        J_proj << _fx / z, 0.0, -_fx * x / (z * z),
            0.0, _fy / z, -_fy * y / (z * z);

        // 2. 相机坐标的反对称矩阵 3 x 3
        Eigen::Matrix3d pc_hat;

        pc_hat << 0.0, -z, y,
            z, 0.0, -x,
            -y, x, 0.0;

        // 3. Pose Jacobian：3 x 6
        Eigen::Matrix<double, 3, 6> J_se3;

        J_se3.block<3, 3>(0, 0).setIdentity(); // 从 J_se3 的第 0 行、第 0 列开始，取出一个 3 x 3 的矩阵块

        J_se3.block<3, 3>(0, 3) = -pc_hat; // 同上

        _jacobianOplusXi = -J_proj * J_se3;

        // 4. Landmark Jacobian：2 x 3
        Eigen::Matrix3d Rcw =
            Tcw.so3().matrix(); // so3 是从 SE3 中取出R, 返回的实际上是 Sophus::SO3d 类型的对象,  matrix 返回一个普通的 eigen 类型， 我们用 eigen 做计算

        _jacobianOplusXj =
            -J_proj * Rcw;
    }

    bool read(std::istream &is) override
    {

        is >> _measurement[0] >> _measurement[1];

        return static_cast<bool>(is);
    }

    bool write(std::ostream &os) const override
    {

        os << _measurement.transpose();

        return static_cast<bool>(os);
    }

private:
    double _fx, _fy, _cx, _cy;
};

int main() {

    // 1. 创建 Pose
    VertexPose pose;

    pose.setEstimate(Sophus::SE3d());


    // 2. 创建 Landmark
    VertexPoint point;

    point.setEstimate(
        Eigen::Vector3d(1.0, 2.0, 5.0)
    );


    // 3. 创建重投影 Edge
    double fx = 500.0;
    double fy = 500.0;
    double cx = 320.0;
    double cy = 240.0;

    EdgeProjection edge(
        fx, fy, cx, cy
    );


    // 4. 连接 Vertex
    edge.setVertex(0, &pose);
    edge.setVertex(1, &point);


    // 5. 设置观测值
    edge.setMeasurement(
        Eigen::Vector2d(420.0, 440.0)
    );

    edge.setInformation(
        Eigen::Matrix2d::Identity()
    );


    // 6. 计算残差
    edge.computeError();

    std::cout << "Error:\n"
              << edge.error()
              << std::endl;

    return 0;
}