#include "TangramSolver.h"

#define DEBUG_MODE 1

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
        cv::approxPolyDP(contours[i], approxPoints, 1, true);
        
        approxContours.push_back(approxPoints);
    }
    
    //对每个轮廓精简轮廓点
    for(int i=0;i<approxContours.size();i++)
        stripContour(approxContours[i]);
    
    //返回polygonPatterns
    polygonPatterns.resize(approxContours.size());
    for(int i=0;i<approxContours.size();i++)
        polygonPatterns[i].setPoints(approxContours[i]);
}

void TangramSolver::stripContour(std::vector<cv::Point> &cntPts)
{
    //遍历轮廓点，删除角度接近180度的点，融合相邻的距离较近的点
    int ptSize = cntPts.size();
    std::vector<cv::Point> cntPts_new;
    for(int i=0;i<cntPts.size();i++)
    {
        //角度
        float angle = calcAngle(cntPts[getPrevIndex(ptSize,i)],cntPts[i],cntPts[getNextIndex(ptSize,i)]);
        //std::cout<<"angle:"<<angle<<std::endl;
        if(std::fabs(angle-CV_PI) < 0.08)
            continue;
        
        //距离
        cv::Point pt1 = cntPts[i];
        cv::Point pt2 = cntPts[getNextIndex(ptSize,i)];
        int distance = std::abs(pt1.x-pt2.x) + std::abs(pt1.y-pt2.y);
        //std::cout<<"distance:"<<distance<<std::endl;
        if(distance < 10)
        {
            //计算交点
            cv::Point p1 = cntPts[getPrevIndex(ptSize,i)];
            cv::Point p2 = cntPts[i];
            cv::Point p3 = cntPts[getNextIndex(ptSize,i)];
            cv::Point p4 = cntPts[getNextIndex(ptSize,getNextIndex(ptSize,i))];
            
            //没有交点就取中点
            cv::Point newPt;
            bool isIntersec = calcInterSec(p1,p2,p3,p4,newPt);
            if(!isIntersec)
            {
                newPt.x = (pt1.x+pt2.x)/2;
                newPt.y = (pt1.y+pt2.y)/2;
            }
            
            cntPts_new.push_back(newPt);
            i++;
            continue;
        }
        
        cntPts_new.push_back(cntPts[i]);
    }
    
    cntPts.assign(cntPts_new.begin(),cntPts_new.end());
}

void TangramSolver::solve(const cv::Mat &unitsImg, const cv::Mat &dstsImg, std::vector<std::vector<cv::Point> > &resultPos)
{
    //double runtime = cv::getTickCount();
    //runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
    //std::cout<<"runtime:"<<runtime<<std::endl;
    
    //预处理之阈值化
    cv::Mat binUnitsImg, binDstsImg;
    myThreshold(unitsImg,binUnitsImg);
    myThreshold(dstsImg,binDstsImg);
    
    //预处理之缩放至图案面积一致
    
    
    //test preprocess
    if(DEBUG_MODE)
    {/*
        cv::namedWindow("pre_binUnitsImg",0);
        cv::imshow("pre_binUnitsImg",binUnitsImg);
        cv::namedWindow("pre_binDstsImg",0);
        cv::imshow("pre_binDstsImg",binDstsImg);
        */
    }
    
    //提取polygon
    std::vector<PolygonPattern> unitPolygons;
    std::vector<PolygonPattern> dstPolygons;
    extractPolygonPatterns(binUnitsImg,unitPolygons);
    extractPolygonPatterns(binDstsImg,dstPolygons);
    
    //test polygon
    if(DEBUG_MODE)
    {
        cv::Mat binUnitsImg_test;
        drawPolygons(binUnitsImg,unitPolygons,binUnitsImg_test);
        cv::namedWindow("binUnitsImg_test",0);
        cv::imshow("binUnitsImg_test",binUnitsImg_test);
        
        cv::Mat binDstsImg_test;
        drawPolygons(binDstsImg,dstPolygons,binDstsImg_test);
        cv::namedWindow("binDstsImg_test",0);
        cv::imshow("binDstsImg_test",binDstsImg_test);
    }
    
    PolygonPattern resultPolygon;
    place(dstPolygons[0],0,unitPolygons[0],0,resultPolygon);
    
    cv::waitKey();
}

void TangramSolver::getRotatedVec(cv::Point2f vecA1,cv::Point2f vecA2,cv::Point2f vecB1,cv::Point2f &vecB2)
{
    float x1 = vecA1.x;
    float y1 = vecA1.y;
    
    float x2 = vecA2.x;
    float y2 = vecA2.y;
    
    float sin_theta = (x2*y1 - x1*y2)/(x1*x1 + y1*y1);
    float cos_theta = (x1*x2 + y1*y2)/(x1*x1 + y1*y1);
    
    float x3 = vecB1.x;
    float y3 = vecB1.y;
    
    vecB2.y = y3*cos_theta - x3*sin_theta;
    vecB2.x = y3*sin_theta + x3*cos_theta;
}

float TangramSolver::getVecNorm(cv::Point2f vec)
{
    return std::sqrt(vec.x*vec.x+vec.y*vec.y);
}

bool TangramSolver::place(PolygonPattern &dstPolygon, int dstCornerId, PolygonPattern &unitPolygon, int unitCornerId, PolygonPattern &resultPolygon)
{
    //以给定角点作为两个图案的原点
    cv::Point2f dstOriginPt = dstPolygon.getCntPoint(dstCornerId);
    cv::Point2f unitOriginPt = unitPolygon.getCntPoint(unitCornerId);
    
    //选定标尺向量
    cv::Point2f unitSideVec0 = unitPolygon.getCntPoint(unitPolygon.getNextCntPointId(unitCornerId)) - unitOriginPt;
    cv::Point2f dstSideVec0 = dstPolygon.getCntPoint(dstPolygon.getNextCntPointId(dstCornerId)) - dstOriginPt;
    dstSideVec0 *= getVecNorm(unitSideVec0)/getVecNorm(dstSideVec0);
    
    //遍历单元块角点，获取新坐标系下的坐标
    int unitCntPtsSize = unitPolygon.getCntPtsSize();
    std::vector<cv::Point2f> newUnitPts(unitCntPtsSize);
    for(int i=0;i<unitCntPtsSize;i++)
    {
        cv::Point2f unitSideVec = unitPolygon.getCntPoint(i) - unitOriginPt;
        cv::Point2f newUnitSideVec;
        getRotatedVec(unitSideVec0,dstSideVec0,unitSideVec,newUnitSideVec);
        
        newUnitPts[i] = dstOriginPt + newUnitSideVec;
    }
    
}

void TangramSolver::drawPolygon(cv::Mat &img, PolygonPattern &polygon)
{
    std::vector<cv::Point2f> contours_2f;
    polygon.getAllCntPoints(contours_2f);
    std::vector<std::vector<cv::Point>> contours(1);
    for(int k=0;k<contours_2f.size();k++)
        contours[0].push_back(cv::Point(contours_2f[k].x,contours_2f[k].y));
    cv::drawContours(img,contours,0,cv::Scalar(255),-1);
}

void TangramSolver::drawPolygons(const cv::Mat &img, std::vector<PolygonPattern> &polygons, cv::Mat &outImg)
{
    img.copyTo(outImg);
    cv::cvtColor(outImg,outImg,CV_GRAY2BGR);
    
    std::cout<<"polygons.size():"<<polygons.size()<<std::endl;
    for(int i=0; i<polygons.size();i++)
    {
        //得到轮廓点
        std::vector<cv::Point2f> contours_2f;
        polygons[i].getAllCntPoints(contours_2f);
        std::cout<<"polygons["<<i<<"]:\n"<<contours_2f<<std::endl;
        //转为point类型
        std::vector<std::vector<cv::Point>> contours(1);
        for(int k=0;k<contours_2f.size();k++)
            contours[0].push_back(cv::Point(contours_2f[k].x,contours_2f[k].y));
        //画出轮廓
        cv::drawContours(outImg,contours,0,cv::Scalar(27,0,250));
        
        int cntPtsSize = polygons[i].getCntPtsSize();
        
        for(int j=0;j<cntPtsSize;j++)
        {
            float angle = polygons[i].getAngle(j);
            angle = angle*180/CV_PI;
            cv::putText(outImg,std::to_string(angle).substr(0,5),polygons[i].getCntPoint(j),0,1,cv::Scalar(200,0,0),1);
            cv::circle(outImg,polygons[i].getCntPoint(j),2,cv::Scalar(0,200,0));
        }
    }
}
