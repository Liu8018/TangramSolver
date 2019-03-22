#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//提取轮廓前的预处理
void myThreshold(const cv::Mat &src, cv::Mat &dst);
void myResize(cv::Mat &img1, cv::Mat &img2, int resizeLength);

//根据三个点的坐标计算角度
float calcAngle(const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Point2f &pt3);

//判断角的凹凸性
bool isConvexCorner(const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Point2f &pt3);

//得到数组中上下元素的下标
int getPrevIndex(int listSize, int currentIndex);
int getNextIndex(int listSize, int currentIndex);

//计算两条边的交点
bool calcInterSec(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt3, const cv::Point &pt4, cv::Point &intersecPt);

//cv::Point2f与cv::Point的相互转换
void point2fToPoint(std::vector<cv::Point2f> ptfs, std::vector<cv::Point> &pts);
void pointToPoint2f(std::vector<cv::Point> pts, std::vector<cv::Point2f> &ptfs);

//由向量A1->A2计算B1->B2
void getRotatedVec(cv::Point2f vecA1,cv::Point2f vecA2,cv::Point2f vecB1,cv::Point2f &vecB2);

//计算向量模
float getVecNorm(cv::Point2f vec);

#endif // FUNCTIONS_H
