#include "PolygonPattern.h"

PolygonPattern::PolygonPattern() 
{
    m_area = -1.0;
}

void PolygonPattern::setPoints(const std::vector<cv::Point> &cntPts)
{
    //转为Point2f，赋给m_cntPts
    for(int i=0;i<cntPts.size();i++)
        m_cntPts.push_back(cv::Point2f(cntPts[i].x,cntPts[i].y));
    
    //给m_angles分配存储空间并初始化
    m_angles.resize(m_cntPts.size());
    for(int i=0;i<m_angles.size();i++)
        m_angles[i] = -1.0;
}

cv::Point2f PolygonPattern::getCntPoint(int pointId)
{
    return m_cntPts[pointId];
}

int PolygonPattern::getPrevCntPointId(int currentPointId)
{
    if(currentPointId == 0)
        return m_cntPts.size()-1;
    else
        return currentPointId-1;
}

int PolygonPattern::getNextCntPointId(int currentPointId)
{
    if(currentPointId == m_cntPts.size()-1)
        return 0;
    else
        return currentPointId+1;
}

int PolygonPattern::getCntPtsSize()
{
    return m_cntPts.size();
}

float PolygonPattern::getAngle(int pointId)
{
    //确保只计算一次
    if(m_angles[pointId] == -1.0)
    {
        //找到该角的三个点
        cv::Point2f pt1 = m_cntPts[getPrevCntPointId(pointId)];
        cv::Point2f pt2 = m_cntPts[pointId];
        cv::Point2f pt3 = m_cntPts[getNextCntPointId(pointId)];
        
        //计算角度
        float angle = calcAngle(pt1,pt2,pt3);
        //根据凹凸性更正角度
        if(!isConvexCorner(pt1,pt2,pt3))
            angle = 2*CV_PI - angle;
        
        //返回值
        m_angles[pointId] = angle;
    }
    
    return m_angles[pointId];
}

float PolygonPattern::getArea()
{
    //确保只计算一次
    if(m_area == -1.0)
        m_area = cv::contourArea(m_cntPts);
    
    return m_area;
}