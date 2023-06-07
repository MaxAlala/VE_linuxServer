#pragma once
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp" // circle
#include <string>
// #include "./Eigen/Dense"
#include "opencv2/objdetect.hpp"
#include <opencv2/dnn.hpp>
#include <iostream>
#include <thread>

class DetectedFaceAndConfidence {
public:
    cv::Point faceCoordinate{0, 0};
    int faceConfidence = 0;

    void clear() {
        faceConfidence = 0;
        faceCoordinate = cv::Point(0, 0);
    }

    DetectedFaceAndConfidence() {
    
    }
    DetectedFaceAndConfidence(cv::Point p) {
        faceCoordinate = p;
    }

    static int loopEnd;
    static int counter;

    static DetectedFaceAndConfidence getFaceWithBiggestConffidence(std::vector<DetectedFaceAndConfidence>& detectedFaces) {

        int faceWithBiggestConfidence = 0;
        for (int i = 0; i < detectedFaces.size(); ++i) {
            //std::cout << "i=" << i << ", detectedFace  x y confidence =" << detectedFaces.at(i).faceCoordinate.x << " " << detectedFaces.at(i).faceCoordinate.y << " " << detectedFaces.at(i).faceConfidence << std::endl;
            if (detectedFaces.at(i).faceConfidence > detectedFaces.at(faceWithBiggestConfidence).faceConfidence) {
                faceWithBiggestConfidence = i;
            }
        }

        std::cout << "biggest i=" << faceWithBiggestConfidence << ", detectedFace  x y confidence ="
            << detectedFaces.at(faceWithBiggestConfidence).faceCoordinate.x << " " << detectedFaces.at(faceWithBiggestConfidence).faceCoordinate.y << " " << detectedFaces.at(faceWithBiggestConfidence).faceConfidence << std::endl;

        return detectedFaces.at(faceWithBiggestConfidence);
    }

    static void increaseFaceConfidence(std::vector<DetectedFaceAndConfidence>& detectedFaces, cv::Point faceToProccess) {

        int x_dif = 70;
        int y_dif = 70;

        for (int i = 0; i < detectedFaces.size(); ++i) {
            if (abs(detectedFaces.at(i).faceCoordinate.x - faceToProccess.x) <= x_dif &&
                abs(detectedFaces.at(i).faceCoordinate.y - faceToProccess.y) <= y_dif) {
                std::cout << " increase face conf to i=" << i << "\n";
                ++(detectedFaces.at(i).faceConfidence);
                std::cout << "conf is =" << detectedFaces.at(i).faceConfidence << "\n";

                return;
            }
        }

        // if detected face doesnt match any existed faces then it should be added 
        detectedFaces.push_back(DetectedFaceAndConfidence(cv::Point(faceToProccess)));
    }

};


class FaceDetector {

    const std::string caffeConfigFile = "/home/pi/Desktop/VE_linuxServer/models/deploy.prototxt";
    const std::string caffeWeightFile = "/home/pi/Desktop/VE_linuxServer/models/res10_300x300_ssd_iter_140000_fp16.caffemodel";
    const std::string tensorflowConfigFile = "/home/pi/Desktop/VE_linuxServer/models/opencv_face_detector.pbtxt";
    const std::string tensorflowWeightFile = "/home/pi/Desktop/VE_linuxServer/models/opencv_face_detector_uint8.pb";
    std::string device = "cpu";
    std::string framework = "caffe";
    cv::dnn::Net net;
    bool isUpdatingFacesThreadStarted = false;
    bool shouldClearVector = false;
public:
    FaceDetector() {
        if (framework == "caffe")
            net = cv::dnn::readNetFromCaffe(caffeConfigFile, caffeWeightFile);
        else
            net = cv::dnn::readNetFromTensorflow(tensorflowWeightFile, tensorflowConfigFile);
    }

    void startUpdatingFaces();
    void stopUpdatingFaces();
    void detectFaceOpenCVDNN(cv::Mat& frameOpenCVDNN, cv::Point& tl, cv::Point& br);
    DetectedFaceAndConfidence currentDetectedFace;
    std::vector<DetectedFaceAndConfidence> detectedFaces;
};



