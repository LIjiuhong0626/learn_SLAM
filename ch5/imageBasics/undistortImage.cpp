#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "./distorted.png";   // 请确保路径正确

int main(int argc, char **argv) {

  // 本程序实现去畸变部分的代码。尽管我们可以调用OpenCV的去畸变，但自己实现一遍有助于理解。
  // 畸变参数
  double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
  // 内参
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

  cv::Mat image = cv::imread(image_file, 0);   // 图像是灰度图，CV_8UC1 ，0：表示以灰度模式读取图片。
  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图

  // 计算去畸变后图像的内容

  // 没有畸变图像 <-  相机坐标系下的点的归一化坐标  -> 畸变图像   (正常情况)
  // 没有畸变图像 ->  相机坐标系下的点的归一化坐标  -> 畸变图像   （在这里）
  for (int v = 0; v < rows; v++) {
    for (int u = 0; u < cols; u++) {
      // 遍历去畸变后图像的理想像素位置 (u,v)
      // 计算它在原始畸变图像中 归一化平面上 对应的采样位置 (u_distorted,v_distorted)

      double x = (u - cx) / fx, y = (v - cy) / fy; // 得到在 相机坐标系下的点的归一化坐标
      double r = sqrt(x * x + y * y);

      // 此时的归一化平面上没有进行畸变， 我们对这个点进行畸变， 最后得到 去畸变图像上的点 和 畸变图像上的点 的对应关系
      double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
      double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
      double u_distorted = fx * x_distorted + cx;
      double v_distorted = fy * y_distorted + cy;



      // 赋值 (最近邻插值)
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
        image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
      } else {
        image_undistort.at<uchar>(v, u) = 0;
      }
    }
  }

  // 画图去畸变后图像
  cv::imshow("distorted", image);
  cv::imshow("undistorted", image_undistort);
  cv::waitKey();
  return 0;
}
