#include "PolygonPattern.h"

PolygonPattern::PolygonPattern() 
{
    m_area = -1.0;
}

void PolygonPattern::setPoints(const std::vector<cv::Point> &cntPts)
{
    /*
    //赋给m_cntPts
    m_cntPts.assign(cntPts.begin(),cntPts.end());
    */
    
    //转为Point2f，赋给m_cntPt2fs
    m_cntPt2fs.resize(cntPts.size());
    for(int i=0;i<cntPts.size();i++)
        m_cntPt2fs[i] = cv::Point2f(cntPts[i].x,cntPts[i].y);
    
    //给m_angles分配存储空间并初始化
    m_angles.resize(m_cntPt2fs.size());
    for(int i=0;i<m_angles.size();i++)
        m_angles[i] = -1.0;
}

void PolygonPattern::setPoint2fs(const std::vector<cv::Point2f> &cntPts)
{
    //赋给m_cntPt2fs
    m_cntPt2fs.assign(cntPts.begin(),cntPts.end());
    
    /*
    //转为Point，赋给m_cntPts
    m_cntPts.resize(cntPts.size());
    for(int i=0;i<cntPts.size();i++)
        m_cntPts.push_back(cv::Point(cntPts[i].x,cntPts[i].y));
    */
        
    //给m_angles分配存储空间并初始化
    m_angles.resize(m_cntPt2fs.size());
    for(int i=0;i<m_angles.size();i++)
        m_angles[i] = -1.0;
}

cv::Point2f PolygonPattern::getCntPoint(int pointId) const
{
    return m_cntPt2fs[pointId];
}

void PolygonPattern::getAllCntPoint2fs(std::vector<cv::Point2f> &cntPts) const
{
    cntPts.assign(m_cntPt2fs.begin(),m_cntPt2fs.end());
}

/*void PolygonPattern::getAllCntPoints(std::vector<cv::Point> &cntPts) const
{
    cntPts.assign(m_cntPts.begin(),m_cntPts.end());
}*/

int PolygonPattern::getPrevCntPointId(int currentPointId) const
{
    return getPrevIndex(m_cntPt2fs.size(),currentPointId);
}

int PolygonPattern::getNextCntPointId(int currentPointId) const
{
    return getNextIndex(m_cntPt2fs.size(),currentPointId);
}

int PolygonPattern::getCntPtsSize() const
{
    return m_cntPt2fs.size();
}

float PolygonPattern::getAngle(int pointId)
{
    //确保只计算一次
    if(m_angles[pointId] == -1.0)
    {
        //找到该角的三个点
        cv::Point2f pt1 = m_cntPt2fs[getPrevCntPointId(pointId)];
        cv::Point2f pt2 = m_cntPt2fs[pointId];
        cv::Point2f pt3 = m_cntPt2fs[getNextCntPointId(pointId)];
        
        //计算角度
        float angle = calcAngle(pt1,pt2,pt3);
        //根据凹凸性更正角度
        if(!isConvexCorner(pt1,pt2,pt3))
            angle = 2*CV_PI - angle;
        
        m_angles[pointId] = angle;
    }
    
    return m_angles[pointId];
}

float PolygonPattern::getArea()
{
    //确保只计算一次
    if(m_area == -1.0)
    {
        cv::Mat canvas(1000,1000,CV_8U,cv::Scalar(0));
        std::vector<std::vector<cv::Point>> contours(1);
        point2fToPoint(m_cntPt2fs,contours[0]);
        cv::drawContours(canvas,contours,0,255,-1);
        
        m_area = cv::countNonZero(canvas);
        //m_area = cv::contourArea(m_cntPt2fs);//有点问题
    }
    
    return m_area;
}
