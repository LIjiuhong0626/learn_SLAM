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
 * 本节演示了如何根据data/目录下的十张图训练字典
 * ************************************************/

int main( int argc, char** argv ) {
    // read the image 
    cout<<"reading images... "<<endl;
    vector<Mat> images; 
    for ( int i=0; i<10; i++ )
    {
        string path = "./data/"+to_string(i+1)+".png";
        images.push_back( imread(path) );
    }
    // detect ORB features
    cout<<"detecting ORB features ... "<<endl;
    Ptr< Feature2D > detector = ORB::create(); // 特征提取算法的基类
    vector<Mat> descriptors; // 保存的是每张图像提取出来的描述子矩阵
    for ( Mat& image:images )
    {
        vector<KeyPoint> keypoints; 
        Mat descriptor; // 一张图片对应一个描述子矩阵，一个矩阵里面又包含很多个描述，每一行是一个描述子有 32 列， 表示 32 byte 共 256 bit
        detector->detectAndCompute( image, Mat(), keypoints, descriptor );
        descriptors.push_back( descriptor ); 
    }
    
    // create vocabulary 
    cout<<"creating vocabulary ... "<<endl;
    DBoW3::Vocabulary vocab;
    vocab.create(descriptors); // 利用之前提取的所有 ORB 描述子来训练视觉词典, 层次聚类树
    cout<<"vocabulary info: "<<vocab<<endl; // DBoW3 内部会遍历这些矩阵，取出其中的每一行描述子，将它们作为层次聚类的训练样本,我们不需要自己把 10 个 Mat 拼接成一个巨大的 Mat 再传入
    vocab.save( "vocabulary.yml.gz" );
    cout<<"done"<<endl;
    
    return 0;
}
