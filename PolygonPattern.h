#ifndef POLYGONPATTERN_H
#define POLYGONPATTERN_H

#include "functions.h"

class PolygonPattern
{
public:
    PolygonPattern();
    
    //访问一些基本属性
    int getCntPtsSize();//轮廓点的数量
    cv::Point2f getCntPoint(int pointId);//随机访问轮廓点
    float getAngle(int pointId);//随机访问角度
    float getArea();//面积
    
    //设置轮廓点集
    void setPoints(const std::vector<cv::Point> &cntPts);
    
private:
    //轮廓点集
    std::vector<cv::Point2f> m_cntPts;
    
    //面积
    float m_area;
    
    //角度
    std::vector<float> m_angles;
    
    //获取某个点的上一个点和下一个点
    int getPrevCntPointId(int currentPointId);
    int getNextCntPointId(int currentPointId);
};

#endif // POLYGONPATTERN_H
