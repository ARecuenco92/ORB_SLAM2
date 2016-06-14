#include "CubeDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <chrono>
#include <ctime>
#include <mutex>

namespace ORB_SLAM2
{


CubeDrawer::CubeDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    // Set Camera Kalibration Matrix
    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fSettings["Camera.fx"];
    K.at<float>(1,1) = fSettings["Camera.fy"];
    K.at<float>(0,2) = fSettings["Camera.cx"];
    K.at<float>(1,2) = fSettings["Camera.cy"];
}

void CubeDrawer::Draw2D(cv::Mat im)
{   
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    if(CanDraw()){
        SetCurrentMapPointsPose();
        DrawAnnotation2D(im);
        DrawCube2D(im);
        DrawpPathToCube2D(im);
    }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;

    /**cout << "Draw2D: "
         << elapsed_seconds.count() * 1000
         << endl;**/
}

void CubeDrawer::Draw3D()
{   
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    if(CanDraw()){
        SetCurrentMapPointsPose();
        DrawAnnotation3D();
        DrawCube3D();
        DrawpPathToCube3D();
    }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;

    /**cout << "Draw3D: "
         << elapsed_seconds.count() * 1000
         << endl;**/
}

void CubeDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    mCameraPose = Tcw.clone();
}

void CubeDrawer::SetCurrentMapPointsPose()
{
    // Get ALl the current MapPoints
    const vector<MapPoint*> &vpRefMPs = mpMap->GetAllMapPoints();

    // Get the current MapPoint position
    pointA = vpRefMPs[0]->GetWorldPos();
    pointB = vpRefMPs[1]->GetWorldPos();
    pointC = vpRefMPs[2]->GetWorldPos();
    
    // B+cross(B-A,B-C);
    pointD = pointA + (pointA - pointB).cross((pointA - pointC));

}

void CubeDrawer::AddNewMapPoint(int index)
{
    // Get ALl the current MapPoints
    const vector<MapPoint*> &vpRefMPs = mpMap->GetAllMapPoints();
    if(vpRefMPs.size() > index){
        cv::Mat pointAux = vpRefMPs[index]->GetWorldPos();
        vpRefMPs[index]->SetWorldPos(vpRefMPs[3]->GetWorldPos());
        vpRefMPs[3]->SetWorldPos(vpRefMPs[2]->GetWorldPos());
        vpRefMPs[2]->SetWorldPos(vpRefMPs[1]->GetWorldPos());
        vpRefMPs[1]->SetWorldPos(pointAux);
    }
}

bool CubeDrawer::CanDraw()
{
    // Get ALl the current MapPoints
    const vector<MapPoint*> &vpRefMPs = mpMap->GetAllMapPoints();

    // Check if the Camera Pose is known & there are at least 3 MapPoints
    return mCameraPose.rows > 0 && vpRefMPs.size() > 2;
}

void CubeDrawer::DrawAnnotation2D(cv::Mat im)
{
    cv::Point2f pt1,pt2,pt3,pt4;
    pt1 = projectPoint(pointA);
    pt2 = projectPoint(pointB);
    pt3 = projectPoint(pointC);
    pt4 = projectPoint(pointD);

    const float r = 5;
    cv::Point2f ptr1,ptr2;
    ptr1.x=pt1.x-r;
    ptr1.y=pt1.y-r;
    ptr2.x=pt1.x+r;
    ptr2.y=pt1.y+r;
    cv::rectangle(im,ptr1,ptr2,cv::Scalar(255,122,0));
    cv::circle(im,pt1,2,cv::Scalar(255,122,0),-1);

    ptr1.x=pt2.x-r;
    ptr1.y=pt2.y-r;
    ptr2.x=pt2.x+r;
    ptr2.y=pt2.y+r;
    cv::rectangle(im,ptr1,ptr2,cv::Scalar(255,122,0));
    cv::circle(im,pt2,2,cv::Scalar(255,122,0),-1);

    ptr1.x=pt3.x-r;
    ptr1.y=pt3.y-r;
    ptr2.x=pt3.x+r;
    ptr2.y=pt3.y+r;
    cv::rectangle(im,ptr1,ptr2,cv::Scalar(255,122,0));
    cv::circle(im,pt3,2,cv::Scalar(255,122,0),-1);

    // Blue, Green, Red
    // Draw the AR Annotation
    cv::line(im,pt1,pt2, cv::Scalar(0,0,255), 3);
    cv::line(im,pt1,pt3, cv::Scalar(0,255,0), 3);
    cv::line(im,pt1,pt4, cv::Scalar(255,0,0), 3);
}
void CubeDrawer::DrawCube2D(cv::Mat im)
{
    // Get the Cube Vertexs
    cv::Mat pointE = unitVector(pointA, pointB);
    cv::Mat pointF = unitVector(pointA, pointC);
    cv::Mat pointG = unitVector(pointA, pointD);

    // Project the points
    cv::Point2f pt1,pt2,pt3,pt4,pt5,pt6,pt7,pt8;
    pt1 = projectPoint(pointA);
    pt2 = projectPoint(pointA + pointE);
    pt3 = projectPoint(pointA + pointF);
    pt4 = projectPoint(pointA + pointE + pointF);

    pt5 = projectPoint(pointA + pointG);
    pt6 = projectPoint(pointA + pointG + pointE);
    pt7 = projectPoint(pointA + pointG + pointF);
    pt8 = projectPoint(pointA + pointG + pointE + pointF);

    // Draw the Cube Edges
    cv::line(im,pt1,pt2, cv::Scalar(20,200,200));
    cv::line(im,pt1,pt3, cv::Scalar(20,200,200));
    cv::line(im,pt3,pt4, cv::Scalar(20,200,200));
    cv::line(im,pt2,pt4, cv::Scalar(20,200,200));

    cv::line(im,pt5,pt6, cv::Scalar(20,200,200));
    cv::line(im,pt5,pt7, cv::Scalar(20,200,200));
    cv::line(im,pt7,pt8, cv::Scalar(20,200,200));
    cv::line(im,pt6,pt8, cv::Scalar(20,200,200));

    cv::line(im,pt1,pt5, cv::Scalar(20,200,200));
    cv::line(im,pt2,pt6, cv::Scalar(20,200,200));
    cv::line(im,pt3,pt7, cv::Scalar(20,200,200));
    cv::line(im,pt4,pt8, cv::Scalar(20,200,200));
}

void CubeDrawer::DrawpPathToCube2D(cv::Mat im)
{
    // Get the centroide of the cube
    cv::Mat pointE = unitVector(pointA, pointB);
    cv::Mat pointF = unitVector(pointA, pointC);
    cv::Mat pointG = unitVector(pointA, pointD);
    cv::Mat center = (pointA + pointE/2 + pointF/2 + pointG/2);

    // Get the Camera Translation
    cv::Mat Rwc(3,3,CV_32F);
    cv::Mat twc(3,1,CV_32F);
    Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
    twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
    twc.at<float>(2) = twc.at<float>(2) + 0.1;

    // Project the points
    cv::Point2f pt1,pt2;
    pt1 = projectPoint(center);
    pt2 = projectPoint(twc);

    // Draw the Path
    cv::arrowedLine(im,pt2,pt1, cv::Scalar(200,20,200), 2);
}

void CubeDrawer::DrawAnnotation3D()
{
    // Draw the AR Annotation
    glPushMatrix();
        glLineWidth(2.5);
        glColor3f(1.0, 0.0, 0.0);
        glBegin(GL_LINES);
        glVertex3f(pointA.at<float>(0),pointA.at<float>(1),pointA.at<float>(2));
        glVertex3f(pointB.at<float>(0),pointB.at<float>(1),pointB.at<float>(2));
        glEnd();

        glColor3f(0.0, 1.0, 0.0);
        glBegin(GL_LINES);
        glVertex3f(pointA.at<float>(0),pointA.at<float>(1),pointA.at<float>(2));
        glVertex3f(pointC.at<float>(0),pointC.at<float>(1),pointC.at<float>(2));
        glEnd();

        glColor3f(0.0, 0.0, 1.0);
        glBegin(GL_LINES);
        glVertex3f(pointA.at<float>(0),pointA.at<float>(1),pointA.at<float>(2));
        glVertex3f(pointD.at<float>(0),pointD.at<float>(1),pointD.at<float>(2));
        glEnd();
    glPopMatrix();
}

void CubeDrawer::DrawCube3D()
{
    // Draw the Cube Vertexs
    cv::Mat pointE = unitVector(pointA, pointB);
    cv::Mat pointF = unitVector(pointA, pointC);
    cv::Mat pointG = unitVector(pointA, pointD);

    cv::Mat pt1 = (pointA);
    cv::Mat pt2 = (pointA + pointE);
    cv::Mat pt3 = (pointA + pointF);
    cv::Mat pt4 = (pointA + pointE + pointF);

    cv::Mat pt5 = (pointA + pointG);
    cv::Mat pt6 = (pointA + pointG + pointE);
    cv::Mat pt7 = (pointA + pointG + pointF);
    cv::Mat pt8 = (pointA + pointG + pointE + pointF);

    // Draw the Cube Edges
    glPushMatrix();
        glColor3f(2.0, 2.0, 0.2);
        glBegin(GL_LINES);
            glVertex3f(pt1.at<float>(0),pt1.at<float>(1),pt1.at<float>(2));
            glVertex3f(pt2.at<float>(0),pt2.at<float>(1),pt2.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt1.at<float>(0),pt1.at<float>(1),pt1.at<float>(2));
            glVertex3f(pt3.at<float>(0),pt3.at<float>(1),pt3.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt3.at<float>(0),pt3.at<float>(1),pt3.at<float>(2));
            glVertex3f(pt4.at<float>(0),pt4.at<float>(1),pt4.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt2.at<float>(0),pt2.at<float>(1),pt2.at<float>(2));
            glVertex3f(pt4.at<float>(0),pt4.at<float>(1),pt4.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt5.at<float>(0),pt5.at<float>(1),pt5.at<float>(2));
            glVertex3f(pt6.at<float>(0),pt6.at<float>(1),pt6.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt5.at<float>(0),pt5.at<float>(1),pt5.at<float>(2));
            glVertex3f(pt7.at<float>(0),pt7.at<float>(1),pt7.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt7.at<float>(0),pt7.at<float>(1),pt7.at<float>(2));
            glVertex3f(pt8.at<float>(0),pt8.at<float>(1),pt8.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt6.at<float>(0),pt6.at<float>(1),pt6.at<float>(2));
            glVertex3f(pt8.at<float>(0),pt8.at<float>(1),pt8.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt1.at<float>(0),pt1.at<float>(1),pt1.at<float>(2));
            glVertex3f(pt5.at<float>(0),pt5.at<float>(1),pt5.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt2.at<float>(0),pt2.at<float>(1),pt2.at<float>(2));
            glVertex3f(pt6.at<float>(0),pt6.at<float>(1),pt6.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt3.at<float>(0),pt3.at<float>(1),pt3.at<float>(2));
            glVertex3f(pt7.at<float>(0),pt7.at<float>(1),pt7.at<float>(2));
        glEnd();
        glBegin(GL_LINES);
            glVertex3f(pt4.at<float>(0),pt4.at<float>(1),pt4.at<float>(2));
            glVertex3f(pt8.at<float>(0),pt8.at<float>(1),pt8.at<float>(2));
        glEnd();
    glPopMatrix();
}

void CubeDrawer::DrawpPathToCube3D()
{
    // Get the centroide of the cube
    cv::Mat pointE = unitVector(pointA, pointB);
    cv::Mat pointF = unitVector(pointA, pointC);
    cv::Mat pointG = unitVector(pointA, pointD);
    cv::Mat center = (pointA + pointE/2 + pointF/2 + pointG/2);

    // Get the Camera Translation
    cv::Mat Rwc(3,3,CV_32F);
    cv::Mat twc(3,1,CV_32F);
    Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
    twc = -Rwc*mCameraPose.rowRange(0,3).col(3);

    // Draw the path
    glPushMatrix ();
        glColor3f(2.0, 0.2, 2.0);
        glBegin(GL_LINES);
            glVertex3f(center.at<float>(0),center.at<float>(1),center.at<float>(2));
            glVertex3f(twc.at<float>(0),twc.at<float>(1),twc.at<float>(2));
        glEnd();

        GLUquadric* cyl = gluNewQuadric();
        glPushMatrix ();
            glTranslatef(center.at<float>(0),center.at<float>(1),center.at<float>(2)-0.05);
            gluCylinder(cyl, 0.02, 0.000, 0.05, 12, 1);   // Cone at end of axis.
        glPopMatrix ();
    glPopMatrix ();
}

cv::Mat CubeDrawer::unitVector(cv::Mat point1, cv::Mat point2){
    cv::Mat point3 = point2 - point1;

    float norm = sqrt(pow(point3.at<float>(0), 2) + pow(point3.at<float>(1), 2) + pow(point3.at<float>(2),2)) * 5;
    point3 = point3 / norm;

    return point3;
}

cv::Point2f CubeDrawer::projectPoint(cv::Mat point){
    // To homogeneous coordinates
    cv::Mat point_HC;
    cv::vconcat(point, cv::Mat(1,1, CV_32F, double(1)), point_HC);

    //
    cv::Mat m1 = cv::Mat(3,4, CV_32F, double(0));
    m1.at<float>(0, 0) = 1;
    m1.at<float>(1, 1) = 1;
    m1.at<float>(2, 2) = 1;

    // Proyect the world point 
    cv::Mat uv = K * m1 * mCameraPose * point_HC;

    // Instanciate the point and return it
    cv::Point2f pt;
    pt.x = uv.at<float>(0) / uv.at<float>(2);
    pt.y = uv.at<float>(1) / uv.at<float>(2);

    return pt;
}

} //namespace ORB_SLAM
