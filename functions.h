#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//提取轮廓前的预处理
void myThreshold(const cv::Mat &src, cv::Mat &dst);
void myScale(cv::Mat &img1, cv::Mat &img2);

//根据三个点的坐标计算角度
float calcAngle(const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Point2f &pt3);

//判断角的凹凸性
bool isConvexCorner(const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Point2f &pt3);

//得到数组中上下元素的下标
int getPrevIndex(int listSize, int currentIndex);
int getNextIndex(int listSize, int currentIndex);

//计算两条边的交点
bool calcInterSec(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt3, const cv::Point &pt4, cv::Point &intersecPt);

#endif // FUNCTIONS_H
