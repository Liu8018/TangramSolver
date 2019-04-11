#include "TangramSolver.h"

#define DEBUG_MODE 0

TangramSolver::TangramSolver() 
{
    m_isFlipEnable = true;
    
    m_resizeLength = 200;
    
    m_distRatio = 0.05;
    m_maxOutDist = 6;
    
    PolygonPattern::setCanvasSize(m_resizeLength);
    
    m_totalCount = 0;
    m_yCount = 0;
    m_nCount = 0;
    m_yDatasetPath = "/media/liu/D/linux-windows/dataset/Tpuzzle/y";
    m_nDatasetPath = "/media/liu/D/linux-windows/dataset/Tpuzzle/n";
}

void TangramSolver::setFlipEnable(bool isFlipEnable)
{
    m_isFlipEnable = isFlipEnable;
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
    /*
    cv::Mat testStripImg(m_resizeLength,m_resizeLength,CV_8U,cv::Scalar(0));
    std::vector<PolygonPattern> plgs(1);
    plgs[0].setPoints(cntPts);
    drawPolygons(plgs,testStripImg);
    cv::namedWindow("testStripImg",0);
    cv::imshow("testStripImg",testStripImg);
    cv::waitKey();
    */
    
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
        float angleDiff = std::fabs(angle-CV_PI);
        if(angleDiff < 0.1)
        {
            isStripped = true;
            continue;
        }
        
        //距离
        cv::Point pt1 = cntPts[i];
        cv::Point pt2 = cntPts[getNextIndex(ptSize,i)];
        int distance = std::abs(pt1.x-pt2.x) + std::abs(pt1.y-pt2.y);
        //std::cout<<"distance:"<<distance<<std::endl;
        
        if(distance < m_resizeLength/40)
        {
            //std::cout<<pt1<<" "<<pt2<<std::endl;
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
            //std::cout<<"newPt:"<<newPt<<std::endl;
            cntPts_new.push_back(newPt);
            
            if(i == cntPts.size()-1)
                cntPts_new.erase(cntPts_new.begin(),cntPts_new.begin()+1);
            
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
        cv::Mat binUnitsImg_test = binUnitsImg.clone();
        drawPolygons(unitPolygons,binUnitsImg_test);
        cv::namedWindow("binUnitsImg_test",0);
        cv::imshow("binUnitsImg_test",binUnitsImg_test);
        
        cv::Mat binDstsImg_test = binDstsImg.clone();
        drawPolygons(dstPolygons,binDstsImg_test);
        cv::namedWindow("binDstsImg_test",0);
        cv::imshow("binDstsImg_test",binDstsImg_test);
    }
    
    if(dstPolygons.empty() || unitPolygons.empty())
        return false;
    
    std::vector<bool> isUsed(unitPolygons.size(),false);
    resultPos.resize(unitPolygons.size());
    bool isFited = depthFirstFit_judge(dstPolygons[0],unitPolygons,isUsed,resultPos);
    
    std::cout<<"total count:"<<m_totalCount<<std::endl;
    std::cout<<"yCount:"<<m_yCount<<std::endl;
    std::cout<<"nCount:"<<m_nCount<<std::endl;
    
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
        if(cv::pointPolygonTest(dstPolygonContour,newUnitPt,1) < -m_maxOutDist)
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
    
    //runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
    //std::cout<<"calcArea runtime:"<<runtime<<std::endl;
    
    //完美放置情况下的剩余面积
    float diffArea = dstArea-unitArea;
    //实际放置后的面积与理想面积之差占单元块面积的比例
    float distRatio = (resultArea - cv::arcLength(tmpResultUnitPos,1) - diffArea)/unitArea;
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
            
            std::cout<<"distRatio:"<<distRatio<<std::endl;
            
            std::vector<std::vector<cv::Point>> newContours(1);
            point2fToPoint(combinedContour2f,newContours[0]);
            /*
            runtime = cv::getTickCount();
            int sArea = scanArea(newContours[0],0,m_resizeLength,0,m_resizeLength);
            runtime = (cv::getTickCount() - runtime) / cv::getTickFrequency();
            std::cout<<"calcArea2 runtime:"<<runtime<<std::endl;
            std::cout<<"scan area: "<<sArea<<std::endl;
            */
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
            
            //test
            cv::Mat testImg(m_resizeLength,m_resizeLength,CV_8UC3,cv::Scalar(0,0,0));
            std::vector<PolygonPattern> testPolygons;
            testPolygons.push_back(resultPolygon);
            drawPolygons(testPolygons,testImg);
            cv::namedWindow("testResultPolygon",0);
            cv::imshow("testResultPolygon",testImg);

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
            for(int flip=0;flip<=m_isFlipEnable;flip++)
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
                            //确定dstPolygon和unitPolygon的情况下，place函数的六个决定性参数：dcId,unitId,flipState,ucId,dcb,ucb
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

bool TangramSolver::depthFirstFit_judge(PolygonPattern &dstPolygon, std::vector<PolygonPattern> &unitPolygons, 
                                    std::vector<bool> isUsed, std::vector<std::vector<cv::Point> > &resultPos)
{
    bool judge = false;
    
    m_totalCount++;
    
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
            for(int flip=0;flip<=m_isFlipEnable;flip++)
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
                            //确定dstPolygon和unitPolygon的情况下，place函数的六个决定性参数：dcId,unitId,flipState,ucId,dcb,ucb
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
                                    {
                                        m_yCount++;
                                        
                                        if(m_yCount%10 == 0)
                                            outputDataset(m_yDatasetPath,dstPolygon,unitPolygons,isUsed);
                                        
                                        judge = true;
                                    }
                                }
                                else
                                {
                                    bool isFited = depthFirstFit_judge(resultPolygon,unitPolygons,nextIsUsed,resultPos);
                                    if(isFited)
                                    {
                                        m_yCount++;
                                        
                                        if(m_yCount%10 == 0)
                                            outputDataset(m_yDatasetPath,dstPolygon,unitPolygons,isUsed);
                                        
                                        judge = true;
                                    }
                                    else
                                    {
                                        m_nCount++;
                                        
                                        if(m_nCount%160 == 0)
                                            outputDataset(m_nDatasetPath,dstPolygon,unitPolygons,isUsed);
                                    }
                                }
                                
                            }
                        }
                    }
                }
            }
            
        }
    }
    
    return judge;
}

void TangramSolver::drawPolygon(cv::Mat &img, PolygonPattern &polygon)
{
    std::vector<cv::Point> contour;
    polygon.getAllCntPoints(contour);
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour);
    
    cv::drawContours(img,contours,0,cv::Scalar(255),-1);
}

void TangramSolver::drawPolygons(std::vector<PolygonPattern> &polygons, cv::Mat &outImg)
{
    if(outImg.channels() == 1)
        cv::cvtColor(outImg,outImg,CV_GRAY2BGR);
    
    outImg = cv::Scalar(0,0,0);
    
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
        cv::drawContours(outImg,contours,0,cv::Scalar(255,255,255),-1);
        
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

void TangramSolver::outputDataset(std::string path, PolygonPattern &dstPolygon, std::vector<PolygonPattern> &unitPolygons, std::vector<bool> isUsed)
{
    cv::Mat outputImg_s1(m_resizeLength,m_resizeLength,CV_8U,cv::Scalar(0));
    cv::Mat outputImg_s2(m_resizeLength,m_resizeLength,CV_8U,cv::Scalar(0));
    drawPolygon(outputImg_s1,dstPolygon);
    for(int u=0;u<unitPolygons.size();u++)
    {
        if(!isUsed[u])
            drawPolygon(outputImg_s2,unitPolygons[u]);
    }
    cv::Mat outputImg(2*m_resizeLength,m_resizeLength,CV_8U);
    outputImg_s1.copyTo(outputImg(cv::Range(0,m_resizeLength),cv::Range(0,m_resizeLength)));
    outputImg_s2.copyTo(outputImg(cv::Range(m_resizeLength,2*m_resizeLength),cv::Range(0,m_resizeLength)));
    writeImg(path,outputImg);
}
