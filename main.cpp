#include "TangramSolver.h"

int main()
{
    //读入单元块
    cv::Mat unitsImg = cv::imread("../TangramSolver/unitPatterns/units7.jpg",0);
    
    //读入目标图像
    cv::Mat dstsImg = cv::imread("../TangramSolver/dstPatterns/d4/02.jpg",0);
    
    TangramSolver solver;
    
    std::vector<std::vector<cv::Point>> resultPos;
    solver.solve(unitsImg,dstsImg,resultPos);
    
    return 0;
}
