// ./build/orb_cv test_1.jpg test_2.jpg
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    cout << "usage: feature_extraction img1 img2" << endl;
    return 1;
  }
  //-- 读取图像
  // OpenCV 4 使用带命名空间的读取模式枚举；读取彩色图像的行为与旧宏一致。
  Mat img_1 = imread(argv[1], cv::IMREAD_COLOR); // 读取为三通道 BGR 彩色图 值是 1
  Mat img_2 = imread(argv[2], cv::IMREAD_COLOR);
  assert(img_1.data != nullptr && img_2.data != nullptr);

  //-- 初始化
  std::vector<KeyPoint> keypoints_1, keypoints_2;
  Mat descriptors_1, descriptors_2;                    // 既可以存储图像，也可以存储描述子矩阵。
  Ptr<FeatureDetector> detector = ORB::create();       // Ptr 是 OpenCV 提供的智能指针模板, 声明一个指针变量 detector，用于指向特征检测器对象。 创建一个 ORB 对象，并用基类接口 FeatureDetector 类型的智能指针管理它。
  Ptr<DescriptorExtractor> descriptor = ORB::create(); // 这是 ORB 类的静态工厂方法，用来创建一个 ORB 算法对象, 本来是彩色图， 在算法内部转换成单通道灰度图
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

  //-- 第一步:检测 Oriented FAST 角点位置
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  detector->detect(img_1, keypoints_1); // 通过指针访问对象的成员函数。
  detector->detect(img_2, keypoints_2); // detect() 得到的不只是角点坐标，还有描述子计算所需要的角度、金字塔层数等信息。

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);
  /*对一个特征点来说，算法会：
  1. 根据 keypoints_1[i].octave，确定对应的图像金字塔尺度。
  2. 根据 keypoints_1[i].pt，确定特征点位置。
  3. 根据 keypoints_1[i].angle，旋转 BRIEF 的采样点对。
  4. 在图像中比较点对的灰度，得到 256 个二进制位（默认参数）。
  5. 将这些位打包为 32 字节，作为该特征点的描述子。
  */
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;

  Mat outimg1;
  drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS); // enum class DrawMatchesFlags, ::：从 DrawMatchesFlags 枚举类型中找到 DRAW_RICH_KEYPOINTS 枚举值。
  // Scalar(255, 0, 0);   // 蓝色（BGR）
  // Scalar::all(x) 表示让四个分量全部等于 x
  // DEFAULT: 对于每个特征点，只画一个小圆圈，主要告诉你特征点在哪里。
  // DRAW_RICH_KEYPOINTS:  圆心：特征点位置。 圆圈直径：特征区域的大小  圆心出发的线段：特征点的方向（当角度有效时）, 会把所有图像金字塔层中检测到的 ORB 特征点，统一绘制到原始图像上，并根据每个特征点的 size 画出不同大小的圆圈。
  // size : 1. 在半径为 15 的圆形区域中计算灰度质心, 2. 计算描述子
  imshow("ORB features", outimg1);


  Mat outimg2;
  drawKeypoints(img_1, keypoints_1, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT); // enum class DrawMatchesFlags, ::：从 DrawMatchesFlags 枚举类型中找到 DRAW_RICH_KEYPOINTS 枚举值。
  imshow("ORB features_DEFAULT", outimg2);
  waitKey(0);


  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  vector<DMatch> matches;
  t1 = chrono::steady_clock::now();
  matcher->match(descriptors_1, descriptors_2, matches); // match() 默认会为第一张图像中的每个描述子，在第二张图像中寻找距离最小的一个描述子。
  t2 = chrono::steady_clock::now();
  time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;
  /*
  matches[0].queryIdx = 5; 第一张图像的第 5 号描述子。
  matches[0].trainIdx = 12; 与第二张图像的第 12 号描述子最相似。
  matches[0].distance = 18; 两者的汉明距离为 18。
  */

  //-- 第四步:匹配点对筛选
  // 计算最小距离和最大距离
  auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const DMatch &m1, const DMatch &m2)
                                { return m1.distance < m2.distance; });
  /*
  返回的是 std::pair，其中存储两个迭代器：
  - first：指向最小元素。
  - second：指向最大元素。*/
  double min_dist = min_max.first->distance;
  double max_dist = min_max.second->distance;

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  std::vector<DMatch> good_matches;
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    if (matches[i].distance <= max(2 * min_dist, 20.0))
    {
      good_matches.push_back(matches[i]);
    }
  }//  matches 为第一张图像中的每个描述子，在第二张图像中寻找汉明距离最小的描述子。
  // 我们希望 保留距离较小的匹配 删除距离过大的匹配  距离多大才算过大 但是假设某次匹配的最小距离特别, 就会有问题
  // max(2 * min_dist, 30.0) 引入了一个经验性的阈值下限 30。

  //-- 第五步:绘制匹配结果
  Mat img_match;
  Mat img_goodmatch;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
  imshow("all matches", img_match);
  imshow("good matches", img_goodmatch);
  waitKey(0);

  return 0;
}
