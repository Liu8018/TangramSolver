#ifndef TANGRAMSOLVER_H
#define TANGRAMSOLVER_H

#include "PolygonPattern.h"

class TangramSolver
{
public:
    TangramSolver();
    
    //包含了预处理、搜索求解的solve()函数
    bool solve(const cv::Mat &unitsImg, const cv::Mat &dstsImg, std::vector<std::vector<cv::Point>> &resultPos);
    
    //设置允许的畸变程度
    void setDistRatio(float distRatio);
    
    //设置是否允许翻转
    void setFlipEnable(bool isFlipEnable);
    
private:
    //是否允许翻转
    int m_isFlipEnable;
    
    //允许的畸变程度
    float m_distRatio;
    
    //缩放大小
    int m_resizeLength;
    
    //目标图案面积
    float m_dstPolygonArea;
    
    //从二值图中提取PolygonPattern
    void extractPolygonPatterns(const cv::Mat &binImg, std::vector<PolygonPattern> &polygonPatterns);
    
    //精简轮廓点
    void stripContour(std::vector<cv::Point> &cntPts);
    
    //放置一个polygon到另一个polygon
    bool place(PolygonPattern &dstPolygon, int dstCornerId, 
               PolygonPattern &unitPolygon, int unitCornerId, 
               bool dcb, bool ucb, 
               PolygonPattern &resultPolygon, std::vector<cv::Point> &resultUnitPos);
    
    //深度优先搜索
    bool depthFirstFit(PolygonPattern &dstPolygon, std::vector<PolygonPattern> &unitPolygons, 
                       std::vector<bool> isUsed, std::vector<std::vector<cv::Point>> &resultPos);
    
    //由向量A1->A2计算B1->B2
    void getRotatedVec(cv::Point2f vecA1,cv::Point2f vecA2,cv::Point2f vecB1,cv::Point2f &vecB2);
    
    //计算向量模
    float getVecNorm(cv::Point2f vec);
    
    //debug用的一些函数
    void drawPolygon(cv::Mat &img, PolygonPattern &polygon);
    void drawPolygons(const cv::Mat &img, std::vector<PolygonPattern> &polygons, cv::Mat &outImg);
};

#endif // TANGRAMSOLVER_H
