#include "TangramSolver.h"

#define DEBUG_MODE 1

TangramSolver::TangramSolver() {}

void TangramSolver::extractPolygonPatterns(const cv::Mat &binImg, std::vector<PolygonPattern> &polygonPatterns)
{
    //提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    //对每个轮廓作多边形拟合
    std::vector<std::vector<cv::Point>> approxContours(contours.size());
    for(int i=0;i<contours.size();i++)
    {
        std::vector<cv::Point> approxPoints;
        cv::approxPolyDP(contours[i], approxPoints, 1, true);
        
        approxContours[i] = approxPoints;
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
    cntPts_new.reserve(cntPts.size());
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
    myScale(binDstsImg,binUnitsImg);
    
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
    
    double runtime = cv::getTickCount();
    
    place(dstPolygons[0],5,unitPolygons[3],1,resultPolygon);
    
    runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
    std::cout<<"runtime:"<<runtime<<std::endl;
    
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
    
    bool dstVecDirect = 1;
    bool unitVecDirect = 1;
    
    //选定标尺向量
    cv::Point2f unitSideVec0;
    cv::Point2f dstSideVec0;
    if(unitVecDirect == 0)
        unitSideVec0 = unitPolygon.getCntPoint(unitPolygon.getNextCntPointId(unitCornerId)) - unitOriginPt;
    else
        unitSideVec0 = unitPolygon.getCntPoint(unitPolygon.getPrevCntPointId(unitCornerId)) - unitOriginPt;
    if(dstVecDirect == 0)
        dstSideVec0 = dstPolygon.getCntPoint(dstPolygon.getNextCntPointId(dstCornerId)) - dstOriginPt;
    else
        dstSideVec0 = dstPolygon.getCntPoint(dstPolygon.getPrevCntPointId(dstCornerId)) - dstOriginPt;
    dstSideVec0 *= getVecNorm(unitSideVec0)/getVecNorm(dstSideVec0);
    
    //遍历单元块角点，获取新坐标系下的坐标，并与目标轮廓接合形成新轮廓
    int dstCntPtsSize = dstPolygon.getCntPtsSize();
    int unitCntPtsSize = unitPolygon.getCntPtsSize();
    
    PolygonPattern tmpResultPolygon;
    std::vector<cv::Point2f> combinedContour2f(dstCntPtsSize+unitCntPtsSize);//存储新轮廓点
    int id = dstCornerId;
    for(int i=0;i<dstCntPtsSize;i++)
    {
        combinedContour2f[i] = dstPolygon.getCntPoint(id);
        
        if(dstVecDirect == 0)
            id = dstPolygon.getNextCntPointId(id);
        else
            id = dstPolygon.getPrevCntPointId(id);
    }
    
    id = unitCornerId;
    for(int i=0;i<unitCntPtsSize;i++)
    {
        cv::Point2f unitSideVec = unitPolygon.getCntPoint(id) - unitOriginPt;
        cv::Point2f newUnitSideVec;
        getRotatedVec(unitSideVec0,dstSideVec0,unitSideVec,newUnitSideVec);
        
        cv::Point2f newUnitPt = dstOriginPt + newUnitSideVec;
        combinedContour2f[dstCntPtsSize+i] = newUnitPt;
        
        if(unitVecDirect == 0)
            id = unitPolygon.getNextCntPointId(id);
        else
            id = unitPolygon.getPrevCntPointId(id);
    }
    tmpResultPolygon.setPoint2fs(combinedContour2f);
    
    //获取目标轮廓和单元块轮廓面积
    //double runtime = cv::getTickCount();
    
    float dstArea = dstPolygon.getArea();
    float unitArea = unitPolygon.getArea();
    float resultArea = tmpResultPolygon.getArea();
    
    //runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
    //std::cout<<"calcArea runtime:"<<runtime<<std::endl;
    
    /*
    std::cout<<"dstArea: "<<dstArea<<std::endl;
    std::cout<<"unitArea: "<<unitArea<<std::endl;
    std::cout<<"dstArea + unitArea: "<<dstArea + unitArea<<std::endl;
    std::cout<<"dstArea - unitArea: "<<dstArea - unitArea<<std::endl;
    std::cout<<"Area: "<<resultArea<<std::endl;
    */
    
    if(resultArea - (dstArea-unitArea) < 500)
    {
        resultPolygon = tmpResultPolygon;
        
        return true;
    }
    else
        return false;
    
/*    
    //test
    std::vector<std::vector<cv::Point>> newContours(1);
    point2fToPoint(combinedContour2f,newContours[0]);

double runtime = cv::getTickCount();
    cv::Mat bg2(1000,1000,CV_8UC3,cv::Scalar(255));
runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
std::cout<<"bg2 runtime:"<<runtime<<std::endl;

runtime = cv::getTickCount();
    bg2 = cv::Scalar(0);
runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
std::cout<<"bg2 to 0 runtime:"<<runtime<<std::endl;

runtime = cv::getTickCount();
    cv::drawContours(bg2,newContours,0,cv::Scalar(255,255,255),-1);
    //cv::erode(bg2,bg2,cv::Mat(3,3,CV_8U,cv::Scalar(1)));
runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
std::cout<<"drawContour runtime:"<<runtime<<std::endl;

    for(int i=0;i<newContours[0].size();i++)
    {
        cv::circle(bg2,newContours[0][i],0,cv::Scalar(255,100,10));
        cv::putText(bg2,std::to_string(i),newContours[0][i],1,1,cv::Scalar(100,200,10));
    }
    cv::namedWindow("bg2",0);
    cv::imshow("bg2",bg2);*/
    
}

void TangramSolver::drawPolygon(cv::Mat &img, PolygonPattern &polygon)
{
    std::vector<cv::Point2f> contours_2f;
    polygon.getAllCntPoint2fs(contours_2f);
    std::vector<std::vector<cv::Point>> contours(1);
    for(int k=0;k<contours_2f.size();k++)
        contours[0].push_back(cv::Point(contours_2f[k].x,contours_2f[k].y));
    cv::drawContours(img,contours,0,cv::Scalar(255));
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
        polygons[i].getAllCntPoint2fs(contours_2f);
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
