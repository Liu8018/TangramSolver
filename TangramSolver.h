#ifndef TANGRAMSOLVER_H
#define TANGRAMSOLVER_H

#include "PolygonPattern.h"

class TangramSolver
{
public:
    TangramSolver();
    
    //包含了预处理、搜索求解的solve()函数
    void solve(const cv::Mat &unitsImg, const cv::Mat &dstsImg, std::vector<std::vector<cv::Point>> &resultPos);
    
private:
    //从二值图中提取PolygonPattern
    void extractPolygonPatterns(const cv::Mat &binImg, std::vector<PolygonPattern> &polygonPatterns);
    
    //纯粹搜索求解的fit()函数
    void fit();
};

#endif // TANGRAMSOLVER_H
