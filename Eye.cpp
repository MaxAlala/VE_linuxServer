/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

 /*
  * File:   Eye.cpp
  * Author: sol
  *
  * Created on January 2, 2021, 11:46 AM
  */

#include "Eye.h"
#include <iostream>
#include <cassert>
#include <opencv2/calib3d/calib3d.hpp>
#include <thread>
#include "./Eigen/Dense"
#include <sstream>
#include <cmath>
#include "InverseForwardKinematicsModel.h"
#include <sstream>
#include <chrono>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <boost/algorithm/string.hpp>
#include "protocol.hpp"
using boost::asio::ip::tcp;
using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;
//#include "popt_pp.h"
//#include <sys/stat.h>
void shared_cout(std::string msg);

static cv::Point chosenPoint;
static bool wasPointChosen;
using namespace std;


vector< vector< cv::Point3f > > object_points;
vector< vector< cv::Point2f > > image_points;
vector< cv::Point2f > corners;
vector< vector< cv::Point2f > > left_img_points;

cv::Mat img, gray;
cv::Size im_size;
cv::Mat K;
cv::Mat D;
int skipFrame = 0;


void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        chosenPoint.x = x;
        chosenPoint.y = y;
        wasPointChosen = true;
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
}

Eye::Eye(int camera_id, StepperMotorController* stepperMotorController_, InverseForwardKinematicsModel* inverseForwardKinematicsModel_, TcpProtocolClient* tcpClient_, bool shouldFlipFrame_, bool hasMovementDetection_) :
    wasPointChosen{ false },
    selectionWindow{ "select a point" },
    selectedPointWindow{ "selected point" },
    shouldFlipFrame{ shouldFlipFrame_ },
    hasMovementDetection{ hasMovementDetection_ },
    stepperMotorController{ stepperMotorController_},
    inverseForwardKinematicsModel{inverseForwardKinematicsModel_},
    tcpClient{ tcpClient_ }

{

    //movementDetector.reset(new MovementDetector());
    distanceToTheCentralPixel = "0";
    shouldReceiveImageFromTCP = true;

    // matrix of 0 = white
    map2d = cv::Mat::ones(1000, 1000, CV_8UC3) * 0;

    if (!shouldReceiveImageFromTCP) {
        cap.open(camera_id);
        if (!cap.isOpened()) {
            assert("ERROR! Unable to open camera\n");
        }
    }

    cv::namedWindow(selectionWindow, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(selectedPointWindow, cv::WINDOW_AUTOSIZE);
    cv::namedWindow("map", cv::WINDOW_AUTOSIZE);
    int width = 1920;
    int height = 1080;
    //cv::resizeWindow(selectionWindow, width, height);
    //cv::resizeWindow(selectedPointWindow, width, height);
    //    cv::resizeWindow("before", width, height);
    //set the callback function for any mouse event
    cv::setMouseCallback(selectionWindow, CallBackFunc, NULL);
}

void Eye::setCurrentLidarDistance(std::string str) {
    distanceToTheCentralPixel = str;
}

double Eye::getCurrentLidarDistance() {
    //std::stringstream ss;
    //ss << distanceToTheCentralPixel;
    //double distance = 0;
    //ss >> distance;
    //std::cout << "current lidar distance is " << distance << std::endl;
    return 0;
}

Eigen::Vector3d Eye::calculateObstacleCoordinate() {

    inverseForwardKinematicsModel->calculateDH(inverseForwardKinematicsModel->getCurrentThetas());
    Eigen::Matrix<double, 4, 4 > DH_4_0;
    DH_4_0 = inverseForwardKinematicsModel->getDH4_0();

    double lidarDistance = getCurrentLidarDistance();

    Eigen::Matrix<double, 3, 3 > R_4_0;
    
    R_4_0 <<
        DH_4_0(0, 0), DH_4_0(0, 1), DH_4_0(0, 2),
        DH_4_0(1, 0), DH_4_0(1, 1), DH_4_0(1, 2),
        DH_4_0(2, 0), DH_4_0(2, 1), DH_4_0(2, 2);

    Eigen::Vector3d forthOriginIn0 = {DH_4_0(0, 3), DH_4_0(1, 3), DH_4_0(2, 3)};

    //point In Forth Coordinate System transoformed in point in 0 coordinate system
    Eigen::Vector3d p4 = {0, 0, lidarDistance};
    Eigen::Vector3d p0 = R_4_0 * p4 + forthOriginIn0;
    std::cout << "calculated obstacle in 0 cs = " << p0.x() << " " << p0.y() << " " << p0.z() << std::endl;
    return p0;
}

// returns a selected point + shows cam output
//12
//void startAutohomingOperation(BufferedAsyncSerial* serialPort) {
//
//}
//
//void startAutohomingOperation(BufferedAsyncSerial* serialPort) {
//    int k = cv::waitKey(10);
//    // H || h = start homing
//    if (k == 72 || k == 104) {
//        std::cout << "In. \n";
//        serialPort->writeString("R H\n");
//        std::string str;
//        int counter = 0;
//        while (1) {
//            //std::cout << "waits homing. \n";
//            try {
//                if (serialPort == NULL)
//                    break;
//
//                str = serialPort->readStringUntil();
//                if (str != "")
//                    cout << str << endl;
//                if (str == "R H.")
//                {
//                    std::cout << "H REACHED.\n";
//                    break;
//                }
//                else {
//                    // repeats a message until confirmation is received.
//                    if (counter == 100) {
//                        counter = 0;
//                        std::cout << "SEND RH AGAIN.\n";
//                        serialPort->writeString("R H\n");
//                    }
//                    else {
//                        ++counter;
//                    }
//                }
//            }
//            catch (boost::system::system_error& e)
//            {
//                cout << "Error: " << e.what() << endl;
//            }
//        }
//    }
//}
void Eye::addPointBefore(cv::Point point) {
    pointsBefore.push_back(point);
}

void Eye::addPointAfter(cv::Point point) {
    pointsAfter.push_back(point);
}

void Eye::drawCircleAtMap2d(cv::Point point, cv::Scalar color) {
    cv::Point mapCenter(500,500);
    cv::circle(map2d, cv::Point(point.x + mapCenter.x, point.y + mapCenter.y), 3, color, 2);
}

void Eye::drawCirclesAtMap2d() {
    cv::Point mapCenter(500, 500);

    for (int i = 0; i < pointsAfter.size(); i++) {
        cv::circle(map2d, cv::Point(pointsAfter.at(i).x + mapCenter.x, pointsAfter.at(i).y + mapCenter.y), 3, (255, 0, 0), 2);
    }

    for (int i = 0; i < pointsBefore.size(); i++) {
        cv::circle(map2d, cv::Point(pointsBefore.at(i).x + mapCenter.x, pointsBefore.at(i).y + mapCenter.y), 3, (0, 0, 255), 2);
    }
}

void Eye::addPointTo2dMap(cv::Point point) {
    map2d.at<char>(point.x, point.y) = 255;
}

void Eye::clean2dMap() {
    map2d = cv::Mat::ones(1000, 1000, CV_8UC3) * 0;
}

bool Eye::isFaceDetected() {
    return !faceDetector.detectedFaces.empty();
}

cv::Point Eye::run() {
    if (shouldReceiveImageFromTCP) {
    } else {
        cap.read(frame);
    }

    if (frame.empty()) {
        assert("ERROR! blank frame grabbed\n");
    }

    cv::circle(frame, cv::Point(320, 240), 10, cv::Scalar(255, 0, 0), 2);
    if (shouldFlipFrame)cv::flip(frame, frame, 1);

    static cv::Point tl;
    static cv::Point br;

    faceDetector.detectFaceOpenCVDNN(frame,tl, br);
    faceDetector.detectFaceOpenCVDNN(frame,tl, br);
    cv::rectangle(frame, tl, br, cv::Scalar(253, 88, 68), 2, 4);
    //currentFaceCoordinate = faceDetector.currentFaceCoordinate;
    //std::cout << "currentFaceCoordinate=" << currentFaceCoordinate << std::endl;
    //static int counterStartReadingLidarDistance = 0;

    //if (counterStartReadingLidarDistance == 0) {
    //    std::thread readingLidarDistance(&Eye::readLidarDistance, this);
    //    readingLidarDistance.detach();
    //    counterStartReadingLidarDistance = 1;
    //}

    //
    //    cv::line(frame,
    //            cv::Point(frame.cols / 4, 0),
    //            cv::Point(frame.cols / 4, 480),
    //            cv::Scalar(0, 0, 0),
    //            3);
    //
    //    cv::line(frame,
    //            cv::Point(frame.cols / 4 * 2, 0),
    //            cv::Point(frame.cols / 4 * 2, 480),
    //            cv::Scalar(0, 0, 0),
    //            3);
    //
    //    cv::line(frame,
    //            cv::Point(frame.cols / 4 * 3, 0),
    //            cv::Point(frame.cols / 4 * 3, 480),
    //            cv::Scalar(0, 0, 0),
    //            3);
    //
    //    cv::line(frame,
    //            cv::Point(0, frame.rows / 4),
    //            cv::Point(640, frame.rows / 4),
    //            cv::Scalar(0, 0, 0),
    //            3);
    //
    //    cv::line(frame,
    //            cv::Point(0, frame.rows / 4 * 2),
    //            cv::Point(640, frame.rows / 4 * 2),
    //            cv::Scalar(0, 0, 0),
    //            3);
    //
    //    cv::line(frame,
    //            cv::Point(0, frame.rows / 4 * 3),
    //            cv::Point(640, frame.rows / 4 * 3),
    //            cv::Scalar(0, 0, 0),
    //            3);


    int axis1HomingDirection = 1;
    std::vector<int> currentThetas = inverseForwardKinematicsModel->getThetas();
    std::vector<int> currentThetasWhereStartIsZero = inverseForwardKinematicsModel->getThetasWhereStartIsZero();

    //if (!(thetas[1] < 87 && thetas[1] > -87)) {
    //    std::cerr << "Theta 1 is out of the range \n";
    //    exit(0);
    //}

    //if ((thetas[0] >= 90 && thetas[0] <= 180) || (thetas[0] >= 181 && thetas[0] <= 270)) {
    //    axis1HomingDirection = -1;
    //}
    //else if ((thetas[0] < 90 && thetas[0] >= -90) || (thetas[0] >= 271 && thetas[0] <= 450)) {
    //    axis1HomingDirection = 1;
    //}
    //else {
    //    std::cerr << "Theta 0 is out of the range \n";
    //    exit(0);
    //}

    std::string str;
    std::stringstream ss;


    for (int i = 0; i < NUMBER_OF_AXES; i++) {
        ss << "th[" << i << "]=" << (int)inverseForwardKinematicsModel->currentThetasWhereStartIsZero[i];
        str = ss.str();
        ss.str("");
        cv::putText(frame, str, cv::Point(350, (i+1)*50), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(226, 43, 138), 2, false);

    }

    // put directions
    cv::putText(frame, "+", cv::Point(50, 240), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(226, 43, 138), 2, false);
    cv::putText(frame, "+", cv::Point(320, 430), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(226, 43, 138), 2, false);

    //ss << "th[0]=" << (int)inverseForwardKinematicsModel->fromRadToGrad(thetas[0]) << " " << "th[1]=" << (int)inverseForwardKinematicsModel->fromRadToGrad(thetas[1]);
    //str = ss.str();
    //ss.str("");

    //ss << "after theta[0]=" << fromRadToGrad(thetas[0]) << " " << "theta[1]=" << fromRadToGrad(thetas[1]) << std::endl;
    //str = ss.str();
    //ss.clear();
    //shared_cout(str);
    //ss << "theta"
    //cv::putText(frame, str, cv::Point(300, 50), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(226, 43, 138), 2, false);

    str.clear();
    str = ss.str();
    
    cv::putText(frame, str, cv::Point(50, 50), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(226, 43, 138), 2, false);

    imshow(selectionWindow, frame);

    int k = cv::waitKey(10);



    //// M || m = starts moving
    //if (k == 77 || k == 109) {
    //    stepperMotorController->setMovementCommand(stepperMotorController->convertDegreesToAngle(-135, 0), stepperMotorController->convertDegreesToAngle(-24, 1));
    //}
 
    //if (hasMovementDetection) {

    //    if(skipFrame != 0)
    //    {
    //        --skipFrame;
    //        std::cout<< skipFrame << std::endl;
    //        return cv::Point(0, 0);
    //    }
    //    
    //    Point pointToReturn(0, 0);
    //    movementDetector->findMovement(frame, pointToReturn);
    //    if (pointToReturn != Point(0, 0))
    //        skipFrame = 15;
    //    std::cout << pointToReturn << std::endl;
    //    return pointToReturn;
    //} else
    cv::Mat flipedMap2d;
    cv::flip(map2d, flipedMap2d, 0);
    imshow("map", flipedMap2d);

    if (::wasPointChosen) {
        cv::circle(frame, ::chosenPoint, 10, cv::Scalar(205, 0, 0), 2);
        ////cv::putText(frame, distanceToTheCentralPixel, cv::Point(50, 50), 0, 1, (0,255,0));

        //cv::putText(frame, distanceToTheCentralPixel, cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(130, 0, 75), 2, false);

        imshow(selectedPointWindow, frame);

        ::wasPointChosen = false;

        return ::chosenPoint;
    }
    else return cv::Point(0, 0);
}

Eye::Eye(const Eye& orig) {
}

Eye::~Eye() {
}

