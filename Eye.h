/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Eye.h
 * Author: sol
 *
 * Created on January 2, 2021, 11:46 AM
 */

#ifndef EYE_H
#define EYE_H
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp" // circle
#include <string>
#include "./Eigen/Dense"
#include "opencv2/objdetect.hpp"
#include <opencv2/dnn.hpp>
#include "FaceDetector.h"
#include "InverseForwardKinematicsModel.h"
//#include "MovementDetector.h"
class StepperMotorController;
class InverseForwardKinematicsModel;
class TcpProtocolClient;

//class FaceDetector {
//
//    const std::string caffeConfigFile = "models/deploy.prototxt";
//    const std::string caffeWeightFile = "models/res10_300x300_ssd_iter_140000_fp16.caffemodel";
//
//    const std::string tensorflowConfigFile = "models/opencv_face_detector.pbtxt";
//    const std::string tensorflowWeightFile = "models/opencv_face_detector_uint8.pb";
//    string device = "cpu";
//    string framework = "caffe";
//    cv::dnn::Net net;
//
//public:
//    void detectFaceOpenCVDNN(cv::dnn::Net net, cv::Mat& frameOpenCVDNN, string framework);
//
//};

class Eye {
public:
    Eye(int camera_id, InverseForwardKinematicsModel* inverseForwardKinematicsModel_, bool shouldFlipFrame = false,  bool hasMovementDetection = false);
    Eye(const Eye& orig);
    virtual ~Eye();
    cv::Point run();
    void addPointTo2dMap(cv::Point point);
    void addPointBefore(cv::Point point);
    void addPointAfter(cv::Point point);
    void drawCirclesAtMap2d();
    void drawCircleAtMap2d(cv::Point point, cv::Scalar color);
    void clean2dMap();
    void setCurrentLidarDistance(std::string str);

    double getCurrentLidarDistance();
    Eigen::Vector3d calculateObstacleCoordinate();
    cv::Point currentDetectedFace;
    bool isFaceDetected();
    FaceDetector faceDetector;
private:
    //std::unique_ptr<MovementDetector> movementDetector;
    bool shouldReceiveImageFromTCP;
    bool hasMovementDetection;
    bool shouldFlipFrame;
    cv::Point chosenPoint;
    bool wasPointChosen;
    cv::VideoCapture cap;
    cv::Mat frame;
    cv::Mat map2d;
    std::vector<cv::Point> pointsBefore;
    std::vector<cv::Point> pointsAfter;
    std::unique_ptr<InverseForwardKinematicsModel> inverseForwardKinematicsModel;
    const std::string selectionWindow;
    const std::string selectedPointWindow;
    std::string lidarDistance;
    //BufferedAsyncSerial* serialPort;
    StepperMotorController* stepperMotorController;
    TcpProtocolClient* tcpClient;
};

#endif /* EYE_H */

