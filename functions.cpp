#include "functions.h"

void preprocess(const cv::Mat &src, cv::Mat &dst)
{
    //要求
    cv::threshold(src,dst,0,255,CV_THRESH_OTSU);
    
    
}

float calcAngle(const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Point2f &pt3)
{
    float x1 = pt1.x-pt2.x,
          y1 = pt1.y-pt2.y,
          x2 = pt3.x-pt2.x,
          y2 = pt3.y-pt2.y;

    float cosTheta = (x1*x2 + y1*y2) / (std::sqrt(x1*x1 + y1*y1) * std::sqrt(x2*x2 + y2*y2));

    //防止反三角函数计算出错
    if(cosTheta >  1.0) cosTheta =  1.0;
    if(cosTheta < -1.0) cosTheta = -1.0;

    return std::acos(cosTheta);
}

bool isConvexCorner(const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Point2f &pt3)
{
    //轮廓点是按逆时针排列的，以此判断凹凸性
    int Xa = pt2.x-pt1.x;
    int Ya = pt2.y-pt1.y;
    int Xb = pt3.x-pt2.x;
    int Yb = pt3.y-pt2.y;
    
    if(Xa*Yb - Xb*Ya > 0)
        return 0;
    else
        return 1;
}
