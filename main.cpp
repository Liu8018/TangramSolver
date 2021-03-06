#include "TangramSolver.h"

int main()
{
    //读入单元块
    cv::Mat unitsImg = cv::imread("../TangramSolver/unitPatterns/units4.jpg",0);
    
    //读入目标图像
    cv::Mat dstImg = cv::imread("../TangramSolver/dstPatterns/d4/09.jpg",0);
    
    //time
    double runtime = cv::getTickCount();
    
    //solve
    TangramSolver solver;
    solver.setFlipEnable(false);
    
    std::vector<std::vector<cv::Point>> resultPos;
    bool isSolved = solver.solve(unitsImg,dstImg,resultPos);
    
    //time
    runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
    std::cout<<"runtime:"<<runtime<<std::endl;
    
    //test
    if(isSolved)
    {
        cv::Mat testImg = dstImg.clone();
        cv::cvtColor(testImg,testImg,cv::COLOR_GRAY2BGR);
        
        cv::drawContours(testImg,resultPos,-1,cv::Scalar(255,100,30),2);
        
        std::string testImgWinName = "testImg";
        cv::namedWindow(testImgWinName,0);
        cv::imshow(testImgWinName,testImg);
        cv::waitKey();
    }
    else
    {
        std::cout<<"failed!"<<std::endl;
    }
    
    return 0;
}
