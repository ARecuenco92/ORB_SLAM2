#ifndef CUBEDRAWER_H
#define CUBEDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace ORB_SLAM2
{

class CubeDrawer
{
public:
    CubeDrawer(Map* pMap, const string &strSettingPath);

    Map* mpMap;
    
    void Draw2D(cv::Mat im);
    void Draw3D();

    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetCurrentMapPointsPose();
    
    void AddNewMapPoint(int index);
private:

	cv::Mat K;
    cv::Mat mCameraPose;

    cv::Mat pointA;
    cv::Mat pointB;
    cv::Mat pointC;
    cv::Mat pointD;

    // Draw functions 
    bool CanDraw();

    void DrawAnnotation2D(cv::Mat im);
    void DrawCube2D(cv::Mat im);
    void DrawpPathToCube2D(cv::Mat im);

    void DrawAnnotation3D();
    void DrawCube3D();
    void DrawpPathToCube3D();

    // Vector & Point functions
    cv::Point2f projectPoint(cv::Mat point);
    cv::Mat unitVector(cv::Mat point1, cv::Mat point2);
};

} //namespace ORB_SLAM

#endif // CUBEDRAWER_H
