#include "FaceDetector.h"
#include <chrono>
int DetectedFaceAndConfidence::counter = 0;
int DetectedFaceAndConfidence::loopEnd = 10;


// clears face vector every x seconds
void FaceDetector::startUpdatingFaces() {
    static int threadStarted = 0;
    if (!isUpdatingFacesThreadStarted) {
        isUpdatingFacesThreadStarted = true;

        auto updatingDetectedFaces = [this]() {
            while (isUpdatingFacesThreadStarted) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                // std::cout << "face point=" << currentDetectedFace.faceCoordinate.x << " " << currentDetectedFace.faceCoordinate.y << std::endl;

                // std::cout << "currentDetectedFace confidence = " << currentDetectedFace.faceConfidence << "\n";
                
                if(!detectedFaces.empty())
                    std::cout << "detectedFaces.size() = " << detectedFaces.size() << "\n";

                shouldClearVector = true;

                //detectedFaces.clear();
                //currentDetectedFace.faceConfidence = 0;
                //DetectedFacesAndConfidence::counter = 0;
            }
        };

        std::thread updatingDetectedFacesThread(updatingDetectedFaces);
        updatingDetectedFacesThread.detach();
    }

}

void FaceDetector::stopUpdatingFaces() {
    isUpdatingFacesThreadStarted = false;
}

void FaceDetector::detectFaceOpenCVDNN(cv::Mat& frameOpenCVDNN, cv::Point& tl_, cv::Point& br_)
{
    const size_t inWidth = 300;
    const size_t inHeight = 300;
    const double inScaleFactor = 1.0;
    const float confidenceThreshold = 0.7;
    const cv::Scalar meanVal(104.0, 177.0, 123.0);

    int frameHeight = frameOpenCVDNN.rows;
    int frameWidth = frameOpenCVDNN.cols;

    cv::Mat inputBlob;
    if (framework == "caffe")
        inputBlob = cv::dnn::blobFromImage(frameOpenCVDNN, inScaleFactor, cv::Size(inWidth, inHeight), meanVal, false, false);
    else
        inputBlob = cv::dnn::blobFromImage(frameOpenCVDNN, inScaleFactor, cv::Size(inWidth, inHeight), meanVal, true, false);

    net.setInput(inputBlob, "data");
    cv::Mat detection = net.forward("detection_out");

    cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());
    //++DetectedFacesAndConfidence::counter;
    
    //static bool isThreadStarted = false;
    //if (!isThreadStarted)
    //{
    //    startUpdatingFaces();
    //    isThreadStarted = true;

    //}


    if (shouldClearVector) {
        detectedFaces.clear();
        shouldClearVector = false;
        currentDetectedFace.clear();
    }

    for (int i = 0; i < detectionMat.rows; i++)
    {
        float confidence = detectionMat.at<float>(i, 2);

        if (confidence > confidenceThreshold)
        {

            int x1 = static_cast<int>(detectionMat.at<float>(i, 3) * frameWidth);
            int y1 = static_cast<int>(detectionMat.at<float>(i, 4) * frameHeight);
            int x2 = static_cast<int>(detectionMat.at<float>(i, 5) * frameWidth);
            int y2 = static_cast<int>(detectionMat.at<float>(i, 6) * frameHeight);
            tl_ = cv::Point(x1, y1);
            br_ = cv::Point(x2, y2);
            cv::Point currentFacePoint(x1 + (x2 - x1) / 2, y1 + (y2 - y1) / 2);
            //std::cout << "currentFace=" << currentFacePoint.x << " " << currentFacePoint.y << "\n";

            DetectedFaceAndConfidence::increaseFaceConfidence(detectedFaces, currentFacePoint);

            currentDetectedFace = DetectedFaceAndConfidence::getFaceWithBiggestConffidence(detectedFaces);

        }
    }

    //if (DetectedFacesAndConfidence::counter == DetectedFacesAndConfidence::loopEnd) {

    //}
}















