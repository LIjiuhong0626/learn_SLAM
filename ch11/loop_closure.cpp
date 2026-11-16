
#include "DBoW3/DBoW3.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

/***************************************************
 * 本节演示如何利用训练好的视觉词典：
 *
 * 1. 将图像的 ORB 描述子转换为 BoW 向量
 * 2. 计算两张图像之间的相似度
 * 3. 建立图像数据库
 * 4. 查询数据库中与指定图像最相似的图像
 *
 * 在 SLAM 中，这些操作是回环检测的重要基础。
 ***************************************************/

int main(int argc, char **argv) {

    // ========================================================
    // 一、加载已经训练好的视觉词典
    // ========================================================

    cout << "reading database" << endl;

    // Vocabulary：视觉词典
    // 从文件加载之前训练好的词典树。
    // 词典包含：
    // 1. 层次聚类树的结构
    // 2. 视觉单词（Visual Words）
    // 3. 每个视觉单词的 IDF 权重
    // 4. TF-IDF 权重策略和相似度评分方式
    DBoW3::Vocabulary vocab("./vocabulary.yml.gz");

    // 也可以加载规模更大的预训练词典
    // DBoW3::Vocabulary vocab("./vocab_larger.yml.gz");

    // 检查词典是否为空
    if (vocab.empty()) {
        cerr << "Vocabulary does not exist." << endl;
        return 1;
    }


    // ========================================================
    // 二、读取 10 张图像
    // ========================================================

    cout << "reading images... " << endl;

    // 每个 Mat 存放一张图像
    vector<Mat> images;

    for (int i = 0; i < 10; i++) {

        // 构造路径：
        // ./data/1.png、./data/2.png、...、./data/10.png
        string path = "./data/" + to_string(i + 1) + ".png";

        // 读取图像，并存入 vector
        images.push_back(imread(path));
    }

    // 注意：
    // 这里用来比较的 10 张图像，也是之前训练词典的图像。
    // 因此实验结果可能偏乐观，存在过拟合的风险。
    // 实际应用中，最好使用独立的图像数据来评价词典。


    // ========================================================
    // 三、提取每张图像的 ORB 描述子
    // ========================================================

    cout << "detecting ORB features ... " << endl;

    // 创建 ORB 特征提取器
    // Feature2D 是 OpenCV 特征提取算法的基类
    // Ptr 是 OpenCV 提供的智能指针
    Ptr<Feature2D> detector = ORB::create();

    // 存储所有图像的描述子矩阵
    //
    // descriptors[i]：第 i 张图像的描述子矩阵
    // descriptors[i].row(j)：第 i 张图像的第 j 个描述子
    //
    // ORB 每个描述子通常为 256 bit，即 32 Byte。
    // 如果一张图像提取了 500 个特征，
    // 对应的 Mat 大小就是 500 × 32，类型为 CV_8U。
    vector<Mat> descriptors;

    // 遍历每一张图像
    // Mat& 表示引用，避免复制整个图像矩阵
    for (Mat &image : images) {

        // 存储当前图像的关键点
        vector<KeyPoint> keypoints;

        // 存储当前图像的 ORB 描述子
        Mat descriptor;

        // 同时完成：
        // 1. detect：检测 ORB 关键点
        // 2. compute：计算 ORB 描述子
        //
        // image：输入图像
        // Mat()：空掩码，不限制特征提取区域
        // keypoints：输出关键点
        // descriptor：输出描述子矩阵
        detector->detectAndCompute(
            image, Mat(), keypoints, descriptor
        );

        // 保存当前图像的描述子矩阵
        descriptors.push_back(descriptor);
    }


    // ========================================================
    // 四、方法一：直接计算两张图像之间的相似度
    // ========================================================

    cout << "comparing images with images " << endl;

    // 遍历第一张待比较图像
    for (int i = 0; i < images.size(); i++) {

        // BowVector = Bag of Words Vector
        // 即词袋向量。
        //
        // 它不是原始的 ORB 描述子矩阵，
        // 而是由视觉单词 ID 和对应权重组成的稀疏向量。
        //
        // 例如：
        // Word ID    Weight
        //   12        0.25
        //   35        0.40
        //   78        0.35
        DBoW3::BowVector v1;

        // transform：转换
        //
        // 将第 i 张图像的所有 ORB 描述子
        // 转换成一个 BoW 向量。
        //
        // 内部过程：
        // 1. 将每个 ORB 描述子沿词典树向下匹配
        // 2. 找到对应的叶节点（视觉单词）
        // 3. 统计视觉单词的出现情况
        // 4. 根据 TF-IDF 等权重策略计算权重
        // 5. 对词袋向量进行归一化
        //
        // 输出结果保存在 v1 中
        vocab.transform(descriptors[i], v1);

        // 遍历第二张图像
        //
        // 从 j=i 开始，而不是 j=0，
        // 是因为图像相似度具有对称性：
        // score(i,j) = score(j,i)
        //
        // 因此不需要重复计算。
        //
        // 注意：j=i 时，会计算图像与自身的相似度。
        for (int j = i; j < images.size(); j++) {

            // 第二张图像的 BoW 向量
            DBoW3::BowVector v2;

            // 将第 j 张图像的 ORB 描述子转换成 BoW 向量
            vocab.transform(descriptors[j], v2);

            // score：计算两个 BoW 向量的相似度
            //
            // 由于词典采用 L1-norm Scoring，
            // 对归一化的非负 BoW 向量，评分可写为：
            //
            // score = 1 - 0.5 * sum(|v1[k] - v2[k]|)
            //
            // 分数通常在 [0,1]：
            // 越接近 1，表示词袋分布越相似
            // 越接近 0，表示词袋分布差异越大
            //
            // 注意：这是视觉单词分布的相似度，
            // 不是几何匹配的正确率。
            double score = vocab.score(v1, v2);

            // 输出图像相似度
            cout << "image " << i
                 << " vs image " << j
                 << " : " << score << endl;
        }

        cout << endl;
    }


    // ========================================================
    // 五、方法二：建立图像数据库，进行相似图像检索
    // ========================================================

    cout << "comparing images with database " << endl;

    // 创建 DBoW3 图像数据库
    //
    // Database(vocab, false, 0)
    //
    // 第一个参数 vocab：
    //   使用已经训练好的视觉词典。
    //
    // 第二个参数 false：
    //   use_di = false
    //   不启用 Direct Index（直接索引）。
    //   直接索引可用于记录图像特征与词典节点的关联，
    //   便于后续特征匹配。
    //
    // 第三个参数 0：
    //   di_levels = 0
    //   直接索引所使用的层级参数。
    //   因为 use_di=false，这里不会实际使用直接索引。
    //
    // 数据库内部利用 Inverted Index（倒排索引）
    // 加快相似图像检索。
    DBoW3::Database db(vocab, false, 0);

    // 将所有图像添加到数据库
    for (int i = 0; i < descriptors.size(); i++) {

        // add：添加图像
        //
        // 这里传入的仍然是当前图像的描述子矩阵，
        // 而不是原始图像，也不是手动构造的 BoW 向量。
        //
        // DBoW3 内部会：
        // 1. 将 ORB 描述子转换成 BoW 向量
        // 2. 为图像分配数据库中的 Entry ID
        // 3. 将 BoW 信息加入倒排索引
        //
        // 注意：
        // Database 保存的是检索所需的特征表示和索引，
        // 不是原始图像的完整像素数据。
        db.add(descriptors[i]);
    }

    // 输出数据库信息
    // 例如数据库中的图像数量、词典配置等
    cout << "database info: " << db << endl;


    // ========================================================
    // 六、从数据库中查询最相似的图像
    // ========================================================

    // 逐张查询
    for (int i = 0; i < descriptors.size(); i++) {

        // QueryResults：查询结果集合
        //
        // 其中包含若干 QueryResult。
        // 每个结果通常包括：
        // 1. Id：数据库中的图像 Entry ID
        // 2. Score：与查询图像的相似度评分
        //
        // 结果按照相似度从高到低排序。
        DBoW3::QueryResults ret;

        // query：查询数据库
        //
        // 参数 1：descriptors[i]
        //         第 i 张图像的 ORB 描述子矩阵
        //
        // 参数 2：ret
        //         用于保存查询结果
        //
        // 参数 3：4
        //         最多返回 4 个最相似的数据库条目
        //
        // 内部利用倒排索引筛选候选图像，
        // 再计算 BoW 相似度并排序。
        //
        // 注意：
        // 因为当前图像本身已经添加到数据库，
        // 所以查询结果通常会包含自身。
        db.query(descriptors[i], ret, 4);

        // 输出检索结果
        //
        // 例如：
        // searching for image 0 returns ...
        //
        // 返回的 ID 是数据库 Entry ID，
        // 不是视觉词典的 Node ID 或 Word ID。
        cout << "searching for image " << i
             << " returns " << ret << endl << endl;
    }

    cout << "done." << endl;

    return 0;
}
