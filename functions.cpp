#include "functions.h"

void myThreshold(const cv::Mat &src, cv::Mat &dst)
{
    //二值化
    cv::threshold(src,dst,0,255,CV_THRESH_OTSU);
    
    //要求白色为前景，黑色为背景。对图像边缘的像素进行采样判断背景颜色
    cv::Mat centerROI = dst(cv::Range(10,dst.rows-10),cv::Range(10,dst.cols-10));
    int nCenterWhitePix = cv::countNonZero(centerROI);
    int nAllWhitePix = cv::countNonZero(dst);
    int nBorderWhitePix = nAllWhitePix - nCenterWhitePix;
    int nBorderAllPix = dst.rows*dst.cols - (dst.rows-20)*(dst.cols-20);
    
    //认为比例超过一半的像素是背景色
    if(nBorderWhitePix/(float)nBorderAllPix > 0.5)
        dst = 255 - dst;
}

void myScale(cv::Mat &img1, cv::Mat &img2)
{
    
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

int getPrevIndex(int listSize, int currentIndex)
{
    if(currentIndex == 0)
        return listSize-1;
    else
        return currentIndex-1;
}
int getNextIndex(int listSize, int currentIndex)
{
    if(currentIndex == listSize-1)
        return 0;
    else
        return currentIndex+1;
}

bool calcInterSec(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt3, const cv::Point &pt4, cv::Point &intersecPt)
{
    float dx12 = pt1.x-pt2.x;
    float dx34 = pt3.x-pt4.x;
    float dy12 = pt1.y-pt2.y;
    float dy34 = pt3.y-pt4.y;
    float dy42 = pt4.y-pt2.y;
    
    float denominator = dx34*dy12 - dx12*dy34;
    if(std::fabs(denominator) < 0.001)
        return false;
    
    float numerator1 = dx34*dy12*pt2.x + dx12*dx34*dy42 - dx12*dy34*pt4.x;
    float numerator2 = (pt3.x*dy42 + pt4.x*(pt2.y-pt3.y) + pt2.x*dy34)*dy12;
    
    intersecPt.x = numerator1/denominator;
    intersecPt.y = numerator2/denominator + pt2.y;
    
    return true;
}
