#include <iostream>
#include <fstream>
#include <vector>

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unistd.h>  // usleep()

using namespace std;
using namespace Eigen;

// 轨迹文件路径
string trajectory_file = "./trajectory.txt"; // 注意， 这里是相对工作目录， 我可以在工作目录下运行一个程序， 只不过不是 ./ 而是 ./build/plotTrajectory

// Eigen::aligned_allocator 用于保证 Eigen 对象的内存对齐
using PoseVector = vector< // 这里的 using 就是起别名
    Isometry3d,
    Eigen::aligned_allocator<Isometry3d> // 使用 Eigen 的对齐内存分配器。
>;

// 绘制轨迹的函数声明
void DrawTrajectory(const PoseVector &poses);

int main(int argc, char **argv) {

    // 保存所有位姿
    // 每个 Isometry3d 都是一个 SE(3) 变换，包含旋转和平移
    PoseVector poses;

    // 打开轨迹文件
    ifstream fin(trajectory_file); // 这里的 fin 就是一个对象， 这里是在调用构造函数

    if (!fin) { // !fin 判断流状态，!fin.is_open() 判断文件是否打开。 这里还没有开始读， 两者是一样的
        cout << "cannot find trajectory file at "
             << trajectory_file << endl;
        return 1;
    }

    // 每行格式：
    // timestamp tx ty tz qx qy qz qw
    // 时间戳 + 平移向量 + 四元数
    double time, tx, ty, tz, qx, qy, qz, qw;

    while (fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw) { // >> 运算符返回输入流对象本身的引用, ((fin >> time) >> tx) >> ty;
      // 整个 while 条件最后仍然得到输入流，随后转换成布尔值进行判断： 读取成功：继续循环。 读取失败或到达文件末尾且无法再读取：循环终止。
      // 注意：>> 默认按空白字符分隔数据，包括空格、制表符、换行符，并不严格限定每次读取一行。
        // 构造 SE(3) 变换
        // 注意：Eigen 四元数构造顺序为 (w, x, y, z)
        Isometry3d Twr(Quaterniond(qw, qx, qy, qz)); // 这里就不用初始化， 直接给你搞好了嘻嘻

        // 设置平移向量
        // 最终 Twr = [R t]
        //            [0 1]
        Twr.pretranslate(Vector3d(tx, ty, tz));

        // 将当前位姿保存到容器
        poses.push_back(Twr);
    }

    cout << "read total " << poses.size()
         << " pose entries" << endl;

    // 调用 Pangolin 绘制轨迹
    DrawTrajectory(poses);

    return 0;
}


/************************************************************
 * 使用 Pangolin + OpenGL 绘制轨迹
 ************************************************************/
void DrawTrajectory(const PoseVector &poses) {

    // ==================== 1. 创建窗口 ====================

    // 创建 1024 × 768 的 Pangolin 窗口
    pangolin::CreateWindowAndBind(
        "Trajectory Viewer", 1024, 768
    );

    // 开启深度测试：近处物体遮挡远处物体
    glEnable(GL_DEPTH_TEST);

    // 开启颜色混合（常用于透明效果）
    glEnable(GL_BLEND);

    // 设置透明度混合方式
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    // ==================== 2. 设置观察相机 ====================

    pangolin::OpenGlRenderState s_cam(

        // 设置投影矩阵
        // 参数：宽、高、fx、fy、cx、cy、近裁剪面、远裁剪面
        pangolin::ProjectionMatrix(
            1024, 768,
            500, 500,
            512, 389,
            0.1, 1000
        ),

        // 设置观察相机的视角
        // 前三个参数：相机位置
        // 中间三个参数：相机看向的位置
        // 最后三个参数：相机的上方向
        pangolin::ModelViewLookAt(
            0, -0.1, -1.8,  // 相机位置
            0, 0, 0,       // 观察目标（原点）
            0, -1.0, 0     // 上方向
        )
    );


    // ==================== 3. 创建显示区域 ====================

    pangolin::View &d_cam = pangolin::CreateDisplay()

        // 设置显示区域占窗口的比例
        // 前四个参数：bottom、top、left、right
        // 最后一个参数：宽高比（负值表示按比例调整）
        .SetBounds(
            0.0, 1.0,
            0.0, 1.0,
            -1024.0f / 768.0f
        )

        // 设置 3D 鼠标交互处理器
        // 支持旋转、平移、缩放视角
        .SetHandler(new pangolin::Handler3D(s_cam));


    // ==================== 4. 循环绘制 ====================

    // 窗口未关闭时持续绘制
    while (!pangolin::ShouldQuit()) {

        // 设置白色背景
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        // 清空颜色缓冲区和深度缓冲区
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 激活相机视角
        d_cam.Activate(s_cam);

        // 设置线宽为 2 像素
        glLineWidth(2);


        // ==================== 5. 绘制各个位姿坐标轴 ====================

        for (size_t i = 0; i < poses.size(); i++) {

            // 当前位姿在世界坐标系下的原点位置
            // 即 Twr 中的平移向量 t
            Vector3d Ow = poses[i].translation();

            // 局部坐标系中的三个轴端点：
            // X = (0.1, 0,   0)
            // Y = (0,   0.1, 0)
            // Z = (0,   0,   0.1)
            //
            // 通过 Twr 将它们变换到世界坐标系：
            // Pw = Rwr * Pr + twr

            Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
            Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
            Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1)); // 这里算的是 相机坐标系下的三个轴向量 在 世界坐标系下 的坐标

            // 开始绘制线段
            // GL_LINES：每两个顶点组成一条线段
            glBegin(GL_LINES);

            // X 轴：红色
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);  // 起点
            glVertex3d(Xw[0], Xw[1], Xw[2]);  // 终点

            // Y 轴：绿色
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);

            // Z 轴：蓝色
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);

            // 结束绘制
            glEnd();
        }


        // ==================== 6. 绘制轨迹连线 ====================

        // 将相邻两个位姿的原点连接起来
        // 注意 i + 1 不能超过数组范围
        for (size_t i = 0; i + 1 < poses.size(); i++) {

            // 设置轨迹为黑色
            glColor3f(0.0, 0.0, 0.0);

            glBegin(GL_LINES);

            // 相邻的两个位姿
            const auto &p1 = poses[i];
            const auto &p2 = poses[i + 1];

            // 两个位姿的平移向量就是它们各自的世界坐标位置
            // 绘制从 p1 到 p2 的线段
            glVertex3d(
                p1.translation()[0],
                p1.translation()[1],
                p1.translation()[2]
            );

            glVertex3d(
                p2.translation()[0],
                p2.translation()[1],
                p2.translation()[2]
            );

            glEnd();
        }


        // ==================== 7. 更新画面 ====================

        // 完成当前帧的绘制并处理窗口事件
        pangolin::FinishFrame();

        // 暂停 5000 微秒，即 5 ms
        usleep(5000);
    }
}