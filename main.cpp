#include "TangramSolver.h"

int main()
{
    //读入单元块
    cv::Mat unitsImg = cv::imread("../TangramSolver/unitPatterns/units4.jpg",0);
    
    //读入目标图像
    cv::Mat dstsImg = cv::imread("../TangramSolver/dstPatterns/d4/01.jpg",0);
    
    TangramSolver solver;
    
    std::vector<std::vector<cv::Point>> resultPos;
    bool isSolved = solver.solve(unitsImg,dstsImg,resultPos);
    
    //test
    if(isSolved)
    {
        cv::Mat testImg = dstsImg.clone();
        cv::cvtColor(testImg,testImg,cv::COLOR_GRAY2BGR);
        
        cv::drawContours(testImg,resultPos,-1,cv::Scalar(255,100,30));
        
        std::string testImgWinName = "testImg";
        cv::namedWindow(testImgWinName,0);
        cv::imshow(testImgWinName,testImg);
        cv::waitKey();
    }
    
    return 0;
}
