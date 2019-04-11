#include "functions.h"
#include <opencv2/opencv.hpp>

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

void myResize(cv::Mat &img1, cv::Mat &img2, int resizeLength)
{
    //计算白色区域面积比例
    float scaleRatio = sqrt(cv::countNonZero(img1)/(float)cv::countNonZero(img2));
    
    //缩放至白色区域面积相等
    cv::resize(img2,img2,cv::Size(),scaleRatio,scaleRatio);
    
    //缩放至最长边为某个值
    int maxSideLength = std::max(std::max(img1.rows,img1.cols),std::max(img2.rows,img2.cols));
    
    if(maxSideLength <= resizeLength)
        return;
    else
    {
        scaleRatio = resizeLength / (float)maxSideLength;
        cv::resize(img1,img1,cv::Size(),scaleRatio,scaleRatio);
        cv::resize(img2,img2,cv::Size(),scaleRatio,scaleRatio);
    }
    
    //阈值化
    cv::threshold(img1,img1,127,255,cv::THRESH_BINARY);
    cv::threshold(img2,img2,127,255,cv::THRESH_BINARY);
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

bool isConvexCorner2(const std::vector<cv::Point> &cntPts, int ptId)
{
    //按邻域面积判断凹凸性
    cv::Point pt = cntPts[ptId];
    
    int dia = 5;
    float area = dia*dia;
    int rad = dia/2;
    
    float wArea = scanArea(cntPts,pt.x-rad,pt.x+rad,pt.y-rad,pt.y+rad);
    
    float ratio = wArea/area;
    //std::cout<<"isConvexCorner2 area:"<<area<<std::endl;
    //std::cout<<"isConvexCorner2 wArea:"<<wArea<<std::endl;
    //std::cout<<"isConvexCorner2 ratio:"<<ratio<<std::endl;
    
    if(ratio < 0.5)
        return true;
    else
        return false;
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

void point2fToPoint(std::vector<cv::Point2f> ptfs, std::vector<cv::Point> &pts)
{
    pts.resize(ptfs.size());
    
    for(int i=0;i<pts.size();i++)
        pts[i] = cv::Point(ptfs[i].x,ptfs[i].y);
}

void pointToPoint2f(std::vector<cv::Point> pts, std::vector<cv::Point2f> &ptfs)
{
    ptfs.resize(pts.size());
    
    for(int i=0;i<ptfs.size();i++)
        ptfs[i] = cv::Point2f(pts[i].x,pts[i].y);
}

void getRotatedVec(cv::Point2f vecA1,cv::Point2f vecA2,cv::Point2f vecB1,cv::Point2f &vecB2)
{
    float x1 = vecA1.x;
    float y1 = vecA1.y;
    
    float x2 = vecA2.x;
    float y2 = vecA2.y;
    
    float sin_theta = (x2*y1 - x1*y2)/(x1*x1 + y1*y1);
    float cos_theta = (x1*x2 + y1*y2)/(x1*x1 + y1*y1);
    
    float x3 = vecB1.x;
    float y3 = vecB1.y;
    
    vecB2.y = y3*cos_theta - x3*sin_theta;
    vecB2.x = y3*sin_theta + x3*cos_theta;
}

float getVecNorm(cv::Point2f vec)
{
    return std::sqrt(vec.x*vec.x+vec.y*vec.y);
}

int scanArea(const std::vector<cv::Point> &cntPts, int w1, int w2, int h1, int h2)
{
    //cv::Mat testImg(500,500,CV_8U,cv::Scalar(0));
    
    int ptSize = cntPts.size();
    int resultArea = 0;
    
    //从上到下扫描
    for(int y0=h1; y0<h2; y0++)
    {
        //遍历每条线段，找交点
        std::vector<int> intersecXs;//存储所有交点的x坐标
        for(int i=0;i<ptSize;i++)
        {
            //线段两端点
            cv::Point pt1 = cntPts[i];
            int pt2Id = getNextIndex(ptSize,i);
            cv::Point pt2 = cntPts[pt2Id];
            
            //跳过没有交点的线段
            if( (pt1.y < y0 && pt2.y < y0) ||
                (pt1.y > y0 && pt2.y > y0) )
                continue;
            
            //计算交点
            cv::Point intersecPt;
            
            //若交点是当前线段第一个端点,则直接计入交点
            if(pt1.y == y0)
                intersecPt = pt1;
            //若交点是当前线段第二个端点，判断这条线段与下一条线段是否穿过扫描线,若未穿过则计入交点
            else if(pt2.y == y0)
            {
                int pt3Id = getNextIndex(ptSize,pt2Id);
                cv::Point pt3 = cntPts[pt3Id];
                
                if((pt2.y-pt1.y)*(pt3.y-pt2.y) < 0)
                    calcInterSec(cv::Point(0,y0),cv::Point(100,y0),pt1,pt2,intersecPt);
                else
                    continue;
            }
            else
                calcInterSec(cv::Point(0,y0),cv::Point(100,y0),pt1,pt2,intersecPt);
            
            intersecXs.push_back(intersecPt.x);
        }
        
        //对交点x坐标进行从小到大的排序
        std::sort(intersecXs.begin(),intersecXs.end());
        
        //计算两两成对的间隔长度，计入面积
        for(int i=0;i<intersecXs.size();i+=2)
        {
            int x1 = intersecXs[i];
            int x2 = intersecXs[i+1];
            
            if(x2 < w1)
                continue;
            if(x1 > w2)
                break;
            
            if(x1 <= w1 && w1 <= x2 && x2 <= w2)
            {
                resultArea += x2 - w1 + 1;
                //cv::line(testImg,cv::Point(w1,y0),cv::Point(x2,y0),cv::Scalar(255));
            }
            else if(w1 <= x1 && x1 <= w2 && w2 <= x2)
            {
                resultArea += w2 - x1 + 1;
                //cv::line(testImg,cv::Point(x1,y0),cv::Point(w2,y0),cv::Scalar(255));
            }
            else if(x1 <= w1 && w2 <= x2)
            {
                resultArea += w2 - w1 + 1;
                //cv::line(testImg,cv::Point(w1,y0),cv::Point(w2,y0),cv::Scalar(255));
            }
            else
            {
                resultArea += x2 - x1 + 1;
                //cv::line(testImg,cv::Point(x1,y0),cv::Point(x2,y0),cv::Scalar(255));
            }
        }
        //cv::namedWindow("scanAreaTestImg",0);
        //cv::imshow("scanAreaTestImg",testImg);
    }
    
    return resultArea;
}

void writeImg(std::string path, const cv::Mat &img)
{
    struct timespec tn;
    clock_gettime(CLOCK_REALTIME, &tn);
    
    if(path[path.length()-1] != '/')
        path += '/';
    std::stringstream ss;
    ss<<tn.tv_sec<<tn.tv_nsec;
    std::string fileName = path + ss.str() + ".jpg";
    
    cv::imwrite(fileName,img);
}
