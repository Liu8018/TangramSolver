#ifndef POLYGONPATTERN_H
#define POLYGONPATTERN_H

#include "functions.h"

class PolygonPattern
{
public:
    PolygonPattern();
    
    //访问一些基本属性
    int getCntPtsSize() const;//轮廓点的数量
    cv::Point2f getCntPoint(int pointId) const;//随机访问轮廓点
    void getAllCntPoints(std::vector<cv::Point2f> &cntPts) const;
    float getAngle(int pointId);//随机访问角度
    float getArea();//面积
    
    //设置轮廓点集
    void setPoints(const std::vector<cv::Point> &cntPts);
    void setPoint2fs(const std::vector<cv::Point2f> &cntPts);
    
    //获取某个点的上一个点和下一个点
    int getPrevCntPointId(int currentPointId) const;
    int getNextCntPointId(int currentPointId) const;
    
private:
    //轮廓点集
    std::vector<cv::Point2f> m_cntPts;
    
    //面积
    float m_area;
    
    //角度
    std::vector<float> m_angles;
};

#endif // POLYGONPATTERN_H
