#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <chrono>
#include <sophus/se3.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;

void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t);

void bundleAdjustment(
    const vector<Point3f> &points_3d,
    const vector<Point3f> &points_2d,
    Mat &R, Mat &t);

void bundleAdjustmentGaussNewton(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t);
/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() override
  {
    _estimate = Sophus::SE3d();
  }

  /// left multiplication on SE3
  virtual void oplusImpl(const double *update) override
  {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  virtual bool read(istream &in) override {}

  virtual bool write(ostream &out) const override {}
};

/// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

  virtual void computeError() override
  {
    const VertexPose *pose = static_cast<const VertexPose *>(_vertices[0]);
    _error = _measurement - pose->estimate() * _point;
  }

  virtual void linearizeOplus() override
  {
    VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = pose->estimate();
    Eigen::Vector3d xyz_trans = T * _point;
    _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
  }

  bool read(istream &in) {}

  bool write(ostream &out) const {}

protected:
  Eigen::Vector3d _point;
};

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    cout << "usage: pose_estimation_3d3d img1 img2 depth1 depth2" << endl;
    return 1;
  }
  //-- 读取图像
  // OpenCV 4 使用带命名空间的读取模式枚举；读取彩色图像的行为与旧宏一致。
  Mat img_1 = imread(argv[1], cv::IMREAD_COLOR);
  Mat img_2 = imread(argv[2], cv::IMREAD_COLOR);

  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
  cout << "一共找到了" << matches.size() << "组匹配点" << endl;

  // 建立3D点
  // IMREAD_UNCHANGED 保留原始深度与通道数，等价于旧的 CV_LOAD_IMAGE_UNCHANGED。

  // 读取两帧深度图（16位无符号整数，单通道）
  Mat depth1 = imread(argv[3], cv::IMREAD_UNCHANGED);
  Mat depth2 = imread(argv[4], cv::IMREAD_UNCHANGED);

  // 相机内参矩阵
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

  // 保存两帧中相互匹配的三维点
  vector<Point3f> pts1, pts2;

  for (DMatch m : matches)
  {
    // 获取匹配特征点在两帧深度图中的深度值
    ushort d1 = depth1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
    ushort d2 = depth2.ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];

    // 跳过深度无效的匹配点
    if (d1 == 0 || d2 == 0)
      continue;

    // 像素坐标转换为归一化相机坐标
    Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
    Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);

    // 将原始深度值转换为米
    float dd1 = float(d1) / 5000.0;
    float dd2 = float(d2) / 5000.0;

    // 根据归一化坐标和深度恢复三维点 (X=xZ, Y=yZ, Z)
    pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));
    pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
  }

  cout << "3d-3d pairs: " << pts1.size() << endl;

  Mat R, t;

  // SVD 初始化
  pose_estimation_3d3d(pts1, pts2, R, t);

  cout << "ICP via SVD results:" << endl;
  cout << "R = " << R << endl;
  cout << "t = " << t << endl;

  // 保存初始位姿
  Mat R_g2o = R.clone();
  Mat t_g2o = t.clone();

  Mat R_gn = R.clone();
  Mat t_gn = t.clone();

  // g2o 优化
  cout << "Calling bundleAdjustment" << endl;
  bundleAdjustment(pts1, pts2, R_g2o, t_g2o);

  // 自定义高斯牛顿优化
  cout << "Calling bundleAdjustmentGaussNewton" << endl;
  bundleAdjustmentGaussNewton(pts1, pts2, R_gn, t_gn);

  // // verify p1 = R * p2 + t
  // for (int i = 0; i < 5; i++)
  // {
  //   cout << "p1 = " << pts1[i] << endl;
  //   cout << "p2 = " << pts2[i] << endl;
  //   cout << "(R*p2+t) = " << R * (Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t
  //        << endl;
  //   cout << endl;
  // }
}

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches)
{
  //-- 初始化
  Mat descriptors_1, descriptors_2;
  // OpenCV 4 继续使用工厂函数创建 ORB，特征提取与匹配流程不变。
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  //-- 第一步:检测 Oriented FAST 角点位置
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);

  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  vector<DMatch> match;
  // BFMatcher matcher ( NORM_HAMMING );
  matcher->match(descriptors_1, descriptors_2, match);

  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;

  // 找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    double dist = match[i].distance;
    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    if (match[i].distance <= max(2 * min_dist, 30.0))
    {
      matches.push_back(match[i]);
    }
  }
}

Point2d pixel2cam(const Point2d &p, const Mat &K)
{
  return Point2d(
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void bundleAdjustmentGaussNewton(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t)
{
  Eigen::Matrix3d R_eigen;
  Eigen::Vector3d t_eigen;

  cv::cv2eigen(R, R_eigen);
  cv::cv2eigen(t, t_eigen);

  double cost = 0.0, lastcost = 0.0;
  const int iterations = 10;
  for (int iter = 0; iter < iterations; iter++)
  {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

    lastcost = cost, cost = 0;
    for (int j = 0; j < pts1.size(); j++)
    {
      Eigen::Vector3d p1(pts1[j].x, pts1[j].y, pts1[j].z);
      Eigen::Vector3d p2(pts2[j].x, pts2[j].y, pts2[j].z);
      Eigen::Vector3d pc = R_eigen * p2 + t_eigen;
      Eigen::Vector3d e = p1 - pc;
      cost += e.squaredNorm();

      Eigen::Matrix<double, 3, 6> J;

      J << -1, 0, 0, 0, -pc[2], pc[1],
          0, -1, 0, pc[2], 0, -pc[0],
          0, 0, -1, -pc[1], pc[0], 0;

      H += J.transpose() * J;
      b += -J.transpose() * e;
    }
    cout << "iter  " << iter << "  ",
    cout << "lastcost  " << lastcost << "  ";
    cout << "cost  " << cost << endl;
    Eigen::Matrix<double, 6, 1> dx;
    dx = H.ldlt().solve(b);

    // 提取平移和旋转增量
    Eigen::Vector3d rho = dx.head<3>();
    Eigen::Vector3d phi = dx.tail<3>();

    // 计算 phi 的反对称矩阵
    Eigen::Matrix3d W;
    W << 0, -phi[2], phi[1],
        phi[2], 0, -phi[0],
        -phi[1], phi[0], 0;

    double theta = phi.norm();

    Eigen::Matrix3d R_inc, V;

    if (theta < 1e-8)
    {
      R_inc = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;

      V = Eigen::Matrix3d::Identity() + 0.5 * W + (1.0 / 6.0) * W * W;
    }
    else
    {
      R_inc = Eigen::Matrix3d::Identity() + sin(theta) / theta * W + (1 - cos(theta)) / (theta * theta) * W * W;

      V = Eigen::Matrix3d::Identity() + (1 - cos(theta)) / (theta * theta) * W + (theta - sin(theta)) / (theta * theta * theta) * W * W;
    }

    // SE(3) 左乘更新
    R_eigen = R_inc * R_eigen;
    t_eigen = R_inc * t_eigen + V * rho;
  }
  cout << R_eigen << "\n"
       << t_eigen;
}

void pose_estimation_3d3d(const vector<Point3f> &pts1,
                          const vector<Point3f> &pts2,
                          Mat &R, Mat &t)
{
  Point3f p1, p2; // center of mass
  int N = pts1.size();
  for (int i = 0; i < N; i++)
  {
    p1 += pts1[i];
    p2 += pts2[i];
  }
  p1 = Point3f(Vec3f(p1) / N);
  p2 = Point3f(Vec3f(p2) / N);
  vector<Point3f> q1(N), q2(N); // remove the center
  for (int i = 0; i < N; i++)
  {
    q1[i] = pts1[i] - p1;
    q2[i] = pts2[i] - p2;
  }

  // compute q1*q2^T
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for (int i = 0; i < N; i++)
  {
    W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
  }
  cout << "W=" << W << endl;

  // SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  cout << "U=" << U << endl;
  cout << "V=" << V << endl;

  Eigen::Matrix3d R_ = U * (V.transpose());
  if (R_.determinant() < 0)
  {
    R_ = -R_;
  }
  Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

  // convert to cv::Mat
  R = (Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2),
       R_(1, 0), R_(1, 1), R_(1, 2),
       R_(2, 0), R_(2, 1), R_(2, 2));
  t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}

void bundleAdjustment(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t)
{
  // 构建图优化，先设定g2o
  typedef g2o::BlockSolverX BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
  // 梯度下降方法，可以从GN, LM, DogLeg 中选
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer; // 图模型
  optimizer.setAlgorithm(solver); // 设置求解器
  optimizer.setVerbose(true);     // 打开调试输出

  // vertex
  VertexPose *pose = new VertexPose(); // camera pose
  pose->setId(0);
  pose->setEstimate(Sophus::SE3d());
  optimizer.addVertex(pose);

  // edges
  for (size_t i = 0; i < pts1.size(); i++)
  {
    EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(
        Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
    edge->setVertex(0, pose);
    edge->setMeasurement(Eigen::Vector3d(
        pts1[i].x, pts1[i].y, pts1[i].z));
    edge->setInformation(Eigen::Matrix3d::Identity());
    optimizer.addEdge(edge);
  }

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

  cout << endl
       << "after optimization:" << endl;
  cout << "T=\n"
       << pose->estimate().matrix() << endl;

  // convert to cv::Mat
  Eigen::Matrix3d R_ = pose->estimate().rotationMatrix();
  Eigen::Vector3d t_ = pose->estimate().translation();
  R = (Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2),
       R_(1, 0), R_(1, 1), R_(1, 2),
       R_(2, 0), R_(2, 1), R_(2, 2));
  t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}
