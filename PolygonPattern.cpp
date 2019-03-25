#include "PolygonPattern.h"

cv::Mat PolygonPattern::polyCanvas = cv::Mat();

PolygonPattern::PolygonPattern() 
{
    m_area = -1.0;
    
    m_flipState = false;
}

void PolygonPattern::setFlipState(bool flipState)
{
    m_flipState = flipState;
    
    if(m_flipState == true && m_cntPt2fs_flip.empty())
    {
        //计算翻转后的轮廓点
        m_cntPt2fs_flip.resize(m_cntPt2fs.size());
        
        for(int i=0;i<m_cntPt2fs.size();i++)
        {
            float newX = polyCanvas.cols - m_cntPt2fs[i].x;
            float newY = m_cntPt2fs[i].y;
            
            m_cntPt2fs_flip[m_cntPt2fs.size()-1-i] = cv::Point2f(newX,newY);
        }
    }
    
}

bool PolygonPattern::getFlipState()
{
    return m_flipState;
}

void PolygonPattern::setPoints(const std::vector<cv::Point> &cntPts)
{
    //转为Point2f，赋给m_cntPt2fs
    pointToPoint2f(cntPts,m_cntPt2fs);
    
    //给m_angles分配存储空间并初始化
    m_angles.resize(m_cntPt2fs.size());
    for(int i=0;i<m_angles.size();i++)
        m_angles[i] = -1.0;
    
    //给m_angles_flip分配存储空间并初始化
    m_angles_flip.resize(m_cntPt2fs.size());
    for(int i=0;i<m_angles_flip.size();i++)
        m_angles_flip[i] = -1.0;
}

void PolygonPattern::setPoint2fs(const std::vector<cv::Point2f> &cntPts)
{
    //赋给m_cntPt2fs
    m_cntPt2fs.assign(cntPts.begin(),cntPts.end());
        
    //给m_angles分配存储空间并初始化
    m_angles.resize(m_cntPt2fs.size());
    for(int i=0;i<m_angles.size();i++)
        m_angles[i] = -1.0;
    
    //给m_angles_flip分配存储空间并初始化
    m_angles_flip.resize(m_cntPt2fs.size());
    for(int i=0;i<m_angles_flip.size();i++)
        m_angles_flip[i] = -1.0;
}

cv::Point2f PolygonPattern::getCntPoint(int pointId) const
{
    if(m_flipState == false)
        return m_cntPt2fs[pointId];
    else
        return m_cntPt2fs_flip[pointId];
}

void PolygonPattern::getAllCntPoint2fs(std::vector<cv::Point2f> &cntPts) const
{
    if(m_flipState == false)
        cntPts.assign(m_cntPt2fs.begin(),m_cntPt2fs.end());
    else
        cntPts.assign(m_cntPt2fs_flip.begin(),m_cntPt2fs_flip.end());
}

void PolygonPattern::getAllCntPoints(std::vector<cv::Point> &cntPts)
{
    if(m_flipState == false)
    {
        //确保只计算一次
        if(m_cntPts.empty())
            point2fToPoint(m_cntPt2fs,m_cntPts);
        
        cntPts.assign(m_cntPts.begin(),m_cntPts.end());
    }
    else
    {
        //确保只计算一次
        if(m_cntPts_flip.empty())
            point2fToPoint(m_cntPt2fs_flip,m_cntPts_flip);
        
        cntPts.assign(m_cntPts_flip.begin(),m_cntPts_flip.end());
    }
}

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
    if(m_flipState == false)
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
            //if(!isConvexCorner(pt1,pt2,pt3))
            //    angle = 2*CV_PI - angle;
            
            if(m_cntPts.empty())
                point2fToPoint(m_cntPt2fs,m_cntPts);
            if(!isConvexCorner2(m_cntPts,pointId))
                angle = 2*CV_PI - angle;
            
            
            m_angles[pointId] = angle;
        }
        
        return m_angles[pointId];
    }
    else
    {
        //确保只计算一次
        if(m_angles_flip.empty())
            m_angles_flip.resize(m_cntPt2fs.size());
        
        if(m_angles_flip[pointId] == -1.0)
        {
            //找到该角的三个点
            cv::Point2f pt1 = m_cntPt2fs_flip[getPrevCntPointId(pointId)];
            cv::Point2f pt2 = m_cntPt2fs_flip[pointId];
            cv::Point2f pt3 = m_cntPt2fs_flip[getNextCntPointId(pointId)];
            
            //计算角度
            float angle = calcAngle(pt1,pt2,pt3);
            //根据凹凸性更正角度
            if(!isConvexCorner(pt1,pt2,pt3))
                angle = 2*CV_PI - angle;
            
            m_angles_flip[pointId] = angle;
        }
        
        return m_angles_flip[pointId];
    }
}

void PolygonPattern::setCanvasSize(int sideLength)
{
    polyCanvas = cv::Mat(sideLength,sideLength,CV_8U,cv::Scalar(0));
}

float PolygonPattern::getArea()
{
    //确保只计算一次
    if(m_area == -1.0)
    {
        polyCanvas = cv::Scalar(0);
        std::vector<std::vector<cv::Point>> contours(1);
        point2fToPoint(m_cntPt2fs,contours[0]);
        cv::drawContours(polyCanvas,contours,0,255,-1);
        
        m_area = cv::countNonZero(polyCanvas);
        
        /*
        if(m_cntPts.empty())
            point2fToPoint(m_cntPt2fs,m_cntPts);
        m_area = scanArea(m_cntPts,0,polyCanvas.cols,0,polyCanvas.rows);
        */
    }
    
    return m_area;
}
