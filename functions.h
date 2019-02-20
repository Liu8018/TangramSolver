#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//提取轮廓前的预处理
void preprocess(const cv::Mat &src, cv::Mat &dst);

//根据三个点的坐标计算角度
float calcAngle(const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Point2f &pt3);

//判断角的凹凸性
bool isConvexCorner(const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Point2f &pt3);

#endif // FUNCTIONS_H
