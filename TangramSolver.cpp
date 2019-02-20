#include "TangramSolver.h"

TangramSolver::TangramSolver() {}

void TangramSolver::extractPolygonPatterns(const cv::Mat &binImg, std::vector<PolygonPattern> &polygonPatterns)
{
    //提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    //对每个轮廓作多边形拟合
    std::vector<std::vector<cv::Point>> approxContours;
    for(int i=0;i<contours.size();i++)
    {
        std::vector<cv::Point> approxPoints;
        cv::approxPolyDP(contours[i], approxPoints, 3, 1);
        
        approxContours.push_back(approxPoints);
    }
    
    //返回值
    polygonPatterns.resize(approxContours.size());
    for(int i=0;i<approxContours.size();i++)
        polygonPatterns[i].setPoints(approxContours[i]);
}

void TangramSolver::solve(const cv::Mat &unitsImg, const cv::Mat &dstsImg, std::vector<std::vector<cv::Point> > &resultPos)
{
    //预处理
    cv::Mat binUnitsImg, binDstsImg;
    preprocess(unitsImg,binUnitsImg);
    preprocess(dstsImg,binDstsImg);
    
    //test preprocess
    cv::namedWindow("pre_binUnitsImg",0);
    cv::imshow("pre_binUnitsImg",binUnitsImg);
    cv::namedWindow("pre_binDstsImg",0);
    cv::imshow("pre_binDstsImg",binDstsImg);
    
    //提取polygon
    std::vector<PolygonPattern> unitPolygons;
    std::vector<PolygonPattern> dstPolygons;
    extractPolygonPatterns(binUnitsImg,unitPolygons);
    extractPolygonPatterns(binDstsImg,dstPolygons);
    
    //test polygon
    std::cout<<"unitPolygons.size():"<<unitPolygons.size()<<std::endl;
    for(int i=0; i<unitPolygons.size();i++)
    {
        float area = unitPolygons[i].getArea();
        int cntPtsSize = unitPolygons[i].getCntPtsSize();
        
        for(int j=0;j<cntPtsSize;j++)
        {
            float angle = unitPolygons[i].getAngle(j);
            angle = angle*180/CV_PI;
            cv::putText(binUnitsImg,std::to_string(angle).substr(0,5),unitPolygons[i].getCntPoint(j),0,1,160,2);
        }
    }
    cv::namedWindow("poly_binUnitsImg",0);
    cv::imshow("poly_binUnitsImg",binUnitsImg);
    
    
    cv::waitKey();
}
