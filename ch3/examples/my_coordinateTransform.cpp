#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main()
{
    Quaterniond q1(0.35, 0.2, 0.3, 0.1);
    Quaterniond q2(-0.5, 0.4, -0.1, 0.2);

    q1.normalize(); // 为啥要归一化捏
    q2.normalize();
    /*
    旋转本身只有三个自由度，而普通四元数有四个实数分量。单位长度约束去掉了一个自由度，使它能够表示三维旋转。
    非单位四元数多出来的就是模长这个自由度。这个模长不对应额外的旋转角度，而是一个尺度系数。
    */

    Vector3d t1(0.3, 0.1, 0.1);
    Vector3d t2(-0.1, 0.5, 0.3);

    cout << q1.matrix() << '\n'; // 这里单位化的原因是 Eigen 官方文档明确要求：使用 toRotationMatrix() 时，四元数必须已经归一化。否则转换结果不受保证。
    cout << q2.matrix() << '\n';

    Isometry3d T1 = Isometry3d::Identity(), T2 = Isometry3d::Identity(); // rotate() 和 pretranslate() 是在已有变换的基础上进行运算，不是直接赋值。 所以要初始化
    T1.rotate(q1.matrix());
    T1.pretranslate(t1);

    T2.rotate(q2.matrix()); // 此外还有 prerotate， translate， 都是乘对应的 变换成 4 * 4 的矩阵
    T2.pretranslate(t2);

    // 或者
    // Isometry3d T1;
    // Isometry3d T2;

    // T1.linear() = q1.toRotationMatrix(); // 这里就是直接赋值 linear()=rotatoin(), translation() 可以直接取值， 也可以赋值
    // T1.translation() = t1;

    // T2.linear() = q2.toRotationMatrix();
    // T2.translation() = t2;

    Vector3d p1(0.5, 0, 0.2), p2;
    p2 = T2 * T1.inverse() * p1; // 用 逆 而不是 转置
    cout << p2;
}
