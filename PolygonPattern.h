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
    void getAllCntPoint2fs(std::vector<cv::Point2f> &cntPts) const;
    void getAllCntPoints(std::vector<cv::Point> &cntPts);
    float getAngle(int pointId);//随机访问角度
    float getArea();//面积
    bool getFlipState();
    
    //设置轮廓点集
    void setPoints(const std::vector<cv::Point> &cntPts);
    void setPoint2fs(const std::vector<cv::Point2f> &cntPts);
    
    //获取某个点的上一个点和下一个点
    int getPrevCntPointId(int currentPointId) const;
    int getNextCntPointId(int currentPointId) const;
    
    //设置画布尺寸
    static void setCanvasSize(int sideLength);
    
    //设置翻转状态
    void setFlipState(bool flipState);
    
private:
    //翻转状态
    bool m_flipState;
    
    //画布(计算面积时用到)
    static cv::Mat polyCanvas;
    
    //轮廓点集
        //正面
    std::vector<cv::Point2f> m_cntPt2fs;
    std::vector<cv::Point> m_cntPts;
        //反面
    std::vector<cv::Point2f> m_cntPt2fs_flip;
    std::vector<cv::Point> m_cntPts_flip;
    
    //面积
    float m_area;
    
    //角度
        //正面
    std::vector<float> m_angles;
        //反面
    std::vector<float> m_angles_flip;
};

#endif // POLYGONPATTERN_H
