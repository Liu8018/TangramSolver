#include "TangramSolver.h"

#define DEBUG_MODE 0

TangramSolver::TangramSolver() 
{
    m_resizeLength = 200;
    
    m_distRatio = 0.02;
    
    PolygonPattern::setCanvasSize(m_resizeLength);
}

void TangramSolver::setDistRatio(float distRatio)
{
    m_distRatio = distRatio;
}

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
    bool isStripped = false;
    
    //遍历轮廓点，删除角度接近180度的点，融合相邻的距离较近的点
    int ptSize = cntPts.size();
    std::vector<cv::Point> cntPts_new;
    cntPts_new.reserve(cntPts.size());
    for(int i=0;i<cntPts.size();i++)
    {
        //角度
        float angle = calcAngle(cntPts[getPrevIndex(ptSize,i)],cntPts[i],cntPts[getNextIndex(ptSize,i)]);
        //std::cout<<"angle:"<<angle<<std::endl;
        if(std::fabs(angle-CV_PI) < 0.1)
        {
            isStripped = true;
            continue;
        }
        
        //距离
        cv::Point pt1 = cntPts[i];
        cv::Point pt2 = cntPts[getNextIndex(ptSize,i)];
        int distance = std::abs(pt1.x-pt2.x) + std::abs(pt1.y-pt2.y);
        //std::cout<<"distance:"<<distance<<std::endl;
        if(distance < 10)
        {
            isStripped = true;
            
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
    
    //递归strip直到不能再strip为止
    if(isStripped)
        stripContour(cntPts);
    else
        return;
}

bool TangramSolver::solve(const cv::Mat &unitsImg, const cv::Mat &dstsImg, std::vector<std::vector<cv::Point> > &resultPos)
{
    //double runtime = cv::getTickCount();
    //runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
    //std::cout<<"runtime:"<<runtime<<std::endl;
    
    //预处理之阈值化
    cv::Mat binUnitsImg, binDstsImg;
    myThreshold(unitsImg,binUnitsImg);
    myThreshold(dstsImg,binDstsImg);
    
    float dstPolygonArea_src = cv::countNonZero(binDstsImg);
    
    //预处理之缩放至图案面积一致
    myResize(binDstsImg,binUnitsImg,m_resizeLength);
    
    m_dstPolygonArea = cv::countNonZero(binDstsImg);
    
    //test preprocess
    if(DEBUG_MODE)
    {
        /*
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
    
    if(dstPolygons.empty() || unitPolygons.empty())
        return false;
    
    std::vector<bool> isUsed(unitPolygons.size(),false);
    resultPos.resize(unitPolygons.size());
    bool isFited = depthFirstFit(dstPolygons[0],unitPolygons,isUsed,resultPos);
    
    if(isFited)
    {
        //把resultPos缩放回去
        cv::Mat resultUnitsCanvas(m_resizeLength,m_resizeLength,CV_8U,cv::Scalar(0));
        cv::drawContours(resultUnitsCanvas,resultPos,-1,cv::Scalar(255),-1);
        //cv::imshow("c",resultUnitsCanvas);
        float newArea = cv::countNonZero(resultUnitsCanvas);
        float ratio = std::sqrt(dstPolygonArea_src/newArea);
        
        for(int i=0;i<resultPos.size();i++)
            for(int j=0;j<resultPos[i].size();j++)
                resultPos[i][j] = cv::Point(resultPos[i][j].x*ratio,resultPos[i][j].y*ratio);
    }
    
    return isFited;
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

bool TangramSolver::place(PolygonPattern &dstPolygon, int dstCornerId, 
                          PolygonPattern &unitPolygon, int unitCornerId, 
                          bool dcb, bool ucb, 
                          PolygonPattern &resultPolygon,std::vector<cv::Point> &resultUnitPos)
{
    //以给定角点作为两个图案的原点
    cv::Point2f dstOriginPt = dstPolygon.getCntPoint(dstCornerId);
    cv::Point2f unitOriginPt = unitPolygon.getCntPoint(unitCornerId);
    
    //四种放置方式
    bool dstVecDirect = dcb;
    bool unitVecDirect = ucb;
    
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
    std::vector<cv::Point> tmpResultUnitPos(unitCntPtsSize);
    std::vector<cv::Point2f> combinedContour2f(dstCntPtsSize+unitCntPtsSize);//存储新轮廓点
    
    std::vector<cv::Point> dstPolygonContour;
    dstPolygon.getAllCntPoints(dstPolygonContour);
    
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
        
        //若这个点在目标图案外太远距离，则放置失败
        if(cv::pointPolygonTest(dstPolygonContour,newUnitPt,1) < -8)
            return false;
        
        tmpResultUnitPos[i] = newUnitPt;
        combinedContour2f[dstCntPtsSize+i] = newUnitPt;
        
        if(unitVecDirect == 0)
            id = unitPolygon.getNextCntPointId(id);
        else
            id = unitPolygon.getPrevCntPointId(id);
    }
    
    //确保轮廓点按逆时针旋转
    if(dcb != 0)
        std::reverse(combinedContour2f.begin(),combinedContour2f.end());
    tmpResultPolygon.setPoint2fs(combinedContour2f);
    
    //获取目标轮廓和单元块轮廓面积
    //double runtime = cv::getTickCount();
    
    float dstArea = dstPolygon.getArea();
    float unitArea = unitPolygon.getArea();
    float resultArea = tmpResultPolygon.getArea();
    
    float diffArea = dstArea-unitArea;
    float distRatio = (resultArea - cv::arcLength(tmpResultUnitPos,1) - diffArea)/m_dstPolygonArea;
    
    //runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
    //std::cout<<"calcArea runtime:"<<runtime<<std::endl;
    
    
    
    //判断是否符合条件
    if(distRatio < m_distRatio)
    {
        resultPolygon = tmpResultPolygon;
        resultUnitPos.assign(tmpResultUnitPos.begin(),tmpResultUnitPos.end());
        
        //test
        if(DEBUG_MODE)
        {
            std::cout<<"dstArea: "<<dstArea<<std::endl;
            std::cout<<"unitArea: "<<unitArea<<std::endl;
            std::cout<<"dstArea + unitArea: "<<dstArea + unitArea<<std::endl;
            std::cout<<"dstArea - unitArea: "<<dstArea - unitArea<<std::endl;
            std::cout<<"Area: "<<resultArea<<std::endl;
            
            std::vector<std::vector<cv::Point>> newContours(1);
            point2fToPoint(combinedContour2f,newContours[0]);
            cv::Mat bg2(m_resizeLength,m_resizeLength,CV_8UC3,cv::Scalar(255));
            bg2 = cv::Scalar(0);
            cv::drawContours(bg2,newContours,0,cv::Scalar(255,255,255),-1);
            for(int i=0;i<newContours[0].size();i++)
            {
                cv::circle(bg2,newContours[0][i],0,cv::Scalar(255,100,10));
                cv::putText(bg2,std::to_string(i),newContours[0][i],1,1,cv::Scalar(100,200,10));
            }
            cv::namedWindow("bg2",0);
            cv::imshow("bg2",bg2);
            std::cout<<distRatio<<std::endl;
            cv::waitKey();
        }
        
        return true;
    }
    else
        return false;
    
}

bool TangramSolver::depthFirstFit(PolygonPattern &dstPolygon, std::vector<PolygonPattern> &unitPolygons, 
                                  std::vector<bool> isUsed, std::vector<std::vector<cv::Point> > &resultPos)
{
    int dstPtsSize = dstPolygon.getCntPtsSize();
    int unitSize = unitPolygons.size();
    
    //目标polygon的每个角点
    for(int dcId=0;dcId<dstPtsSize;dcId++)
    {
        //每个单元块
        for(int unitId=0;unitId<unitSize;unitId++)
        {
            //若已使用过则跳过
            if(isUsed[unitId])
                continue;
            
            //正反两面
            for(int flip=0;flip<=1;flip++)
            {
                unitPolygons[unitId].setFlipState(flip);
                
                //单元块的每个角点
                int unitPtsSize = unitPolygons[unitId].getCntPtsSize();
                
                for(int ucId=0;ucId<unitPtsSize;ucId++)
                {                    
                    //若目标角点小于单元块该角点则跳过
                    if(dstPolygon.getAngle(dcId) < unitPolygons[unitId].getAngle(ucId) - 0.1)
                        continue;
                    
                    //四种角点对齐方式
                    for(int dcb=0;dcb<=1;dcb++)
                    {
                        for(int ucb=0;ucb<=1;ucb++)
                        {
                            if(DEBUG_MODE)
                            {
                                std::cout<<"dcId:"<<dcId<<std::endl;
                                std::cout<<"unitId:"<<unitId<<std::endl;
                                std::cout<<"flip:"<<flip<<std::endl;
                                std::cout<<"ucId:"<<ucId<<std::endl;
                                std::cout<<"dcb:"<<dcb<<std::endl;
                                std::cout<<"ucb:"<<ucb<<std::endl;
                            }
                            
                            //尝试放置
                            PolygonPattern resultPolygon;
                            bool isPlaced = place(dstPolygon,dcId,unitPolygons[unitId],ucId,dcb,ucb,resultPolygon,resultPos[unitId]);
                            //若放置成功
                            if(isPlaced)
                            {
                                std::vector<bool> nextIsUsed;
                                nextIsUsed.assign(isUsed.begin(),isUsed.end());
                                
                                nextIsUsed[unitId] = true;
                                
                                //检查是否所有单元块都放置完毕
                                int nUsed=0;
                                for(int i=0;i<unitSize;i++)
                                {
                                    nUsed += nextIsUsed[i];
                                }
                                
                                //若放置完毕且最终图案面积足够小则终止递归,否则继续
                                if(nUsed == unitSize)
                                {
                                    int totalLength = 0;
                                    for(int l=0;l<unitSize;l++)
                                    {
                                        std::vector<cv::Point> tmpUnitContour;
                                        unitPolygons[l].getAllCntPoints(tmpUnitContour);
                                        totalLength += cv::arcLength(tmpUnitContour,1);
                                    }
                                    
                                    float resultAreaRatio = (resultPolygon.getArea()-totalLength)/m_dstPolygonArea;
                                    
                                    if(DEBUG_MODE)
                                        std::cout<<"result area ratio:"<<(resultPolygon.getArea()-totalLength)/m_dstPolygonArea<<"-----------------------------"<<std::endl;
                                    
                                    if(resultAreaRatio < 0.01)
                                        return true;
                                }
                                else
                                {
                                    bool isFited = depthFirstFit(resultPolygon,unitPolygons,nextIsUsed,resultPos);
                                    if(isFited)
                                        return true;
                                }
                                
                            }
                        }
                    }
                }
            }
            
        }
    }
    
    return false;
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
            cv::putText(outImg,std::to_string(angle).substr(0,5),polygons[i].getCntPoint(j),0,0.5,cv::Scalar(200,0,0),1);
            cv::circle(outImg,polygons[i].getCntPoint(j),2,cv::Scalar(0,200,0));
        }
    }
}
