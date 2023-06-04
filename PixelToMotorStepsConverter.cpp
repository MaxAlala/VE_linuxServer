/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PixelToMotorStepsConverter.cpp
 * Author: sol
 * 
 * Created on January 1, 2021, 8:03 PM
 */

#include "PixelToMotorStepsConverter.h"
#include <iostream>
#include "InverseForwardKinematicsModel.h"
PixelToMotorStepsConverter::PixelToMotorStepsConverter(int x_resolution_, int y_resolution_, int focal_length_, int stepsPer1AngleX_, int stepsPer1AngleY_, double stepsPerPixelX_, double stepsPerPixelY_)
:
x_resolution{x_resolution_},
y_resolution{y_resolution_},
focal_length{focal_length_},
angleX{0},
angleY{0},
stepsPer1AngleX{stepsPer1AngleX_},
stepsPer1AngleY{stepsPer1AngleY_},
stepsPerPixelY{stepsPerPixelY_},
stepsPerPixelX{stepsPerPixelX_}
{
}

PixelToMotorStepsConverter::PixelToMotorStepsConverter(const PixelToMotorStepsConverter& orig) {
}

PixelToMotorStepsConverter::~PixelToMotorStepsConverter() {
}

double PixelToMotorStepsConverter::removeSign(double x) {
    return sqrt(x * x);
}

int PixelToMotorStepsConverter::fromRadToGrad(double x) {
    return x * 180 / 3.1415;
}

// idea == we have coordinates, we need rotate along x and y
// --F----
//      /Angle
//     /
//    /
//   /
//  /
// /
///Y
//
//
//using a focal length and x or y pixel coordinate we can calculate an angle to set a detected point to the center of a frame

void PixelToMotorStepsConverter::calculateAnglesUsingLogic(cv::Point& detectedObject, int& angleX, int& angleY) {

    double changer = 0.5;
    angleX = changer*fromRadToGrad(atan((removeSign(x_resolution / 2 - detectedObject.x) / focal_length)));
    angleY = changer*fromRadToGrad(atan((removeSign(y_resolution / 2 - detectedObject.y) / focal_length)));
    if (detectedObject.x > x_resolution / 2) angleX = -angleX;
    if (detectedObject.y > y_resolution / 2) angleY = -angleY;
    std::cout << "calculated x y angles to reach position: " << angleX << " " << angleY << std::endl;
}

void PixelToMotorStepsConverter::calculateStepsByPixel(cv::Point& detectedObject, int& stepsForFirstMotor, int& stepsForSecondMotor) {
    int angle1;
    int angle2;
    calculateAnglesUsingLogic(detectedObject, angle1, angle2);
    float calibrationCoef = 0.8;
    angle1 *= calibrationCoef;
    angle2 *= calibrationCoef;
    stepsForFirstMotor = angle1 * stepsPer1AngleX;
    stepsForSecondMotor = angle2 * stepsPer1AngleY;
    std::cout << "calculated steps " << stepsForFirstMotor << " " << stepsForSecondMotor << std::endl;

}

void PixelToMotorStepsConverter::calculateStepsUsingCalibratedValue(cv::Point& detectedObject, int& stepsForFirstMotor, int& stepsForSecondMotor) {

    int xPixelNumber = abs(x_resolution / 2 - detectedObject.x);
    int yPixelNumber = abs(y_resolution / 2 - detectedObject.y);

    std::cout << "x and y pixel number:" << xPixelNumber << " " << yPixelNumber << std::endl;

    //    
    int howMuchToAddX = 0;

    int howMuchToAddY = 0;
    double multiplierX = 1.1;
    double multiplierY = 1.0;
    //    double multiplierX = 1.9;
    //    double multiplierY = 1.7;
    int pixelsIn1mm = 55;
    int divider = 100;
    int horizontExtraPixelsFor100steps = 27.5 / 4;
    int verticalExtraPixelsFor100steps = 36.7 / 2;

    stepsForFirstMotor = stepsPerPixelX * xPixelNumber * multiplierX;
    stepsForSecondMotor = stepsPerPixelY * yPixelNumber * multiplierY;
    howMuchToAddX = stepsForFirstMotor / divider;
    howMuchToAddY = stepsForSecondMotor / divider;
    std::cout << "howMuchToAdd: " << howMuchToAddX << " " << howMuchToAddY << "\n";
    std::cout << "additional steps:" << horizontExtraPixelsFor100steps * howMuchToAddX
            << " " << verticalExtraPixelsFor100steps * howMuchToAddY << "\n";
    //    stepsForFirstMotor += verticalExtraPixelsFor100steps*howMuchToAddY;
    //    stepsForSecondMotor += horizontExtraPixelsFor100steps*howMuchToAddX;

    // if y goes top *0.8 == descrease value
    //    if (detectedObject.y < y_resolution / 2)
    //        stepsForSecondMotor *= 0.8;

    //    int cellXMultiplier = 1;
    //    int cellYMultiplier = 1;
    //    if (
    //
    //            (detectedObject.x < x_resolution / 4 * 1)
    //            &&
    //            (detectedObject.y < y_resolution / 4 * 1)
    //            ) {
    //        std::cout << "CELL 1 \n";
    //        stepsForSecondMotor = stepsPerPixelY * yPixelNumber * 0.75;
    //    }
    //
    //    if (
    //            (detectedObject.x < x_resolution / 4 * 1)
    //            &&
    //            (detectedObject.y < y_resolution / 4 * 2)
    //            &&
    //            (detectedObject.y > y_resolution / 4 * 1)
    //            ) {
    //        std::cout << "CELL 2 \n";
    //        stepsForFirstMotor = stepsPerPixelX * xPixelNumber * 0.95;
    //        stepsForSecondMotor = stepsPerPixelY * yPixelNumber * 0.72;
    //
    //    }
    //    if (
    //            (detectedObject.x < x_resolution / 4 * 1)
    //            &&
    //            (detectedObject.y < y_resolution / 4 * 3)
    //            &&
    //            (detectedObject.y > y_resolution / 4 * 2)
    //            ) {
    //        std::cout << "CELL 3 \n";
    //        stepsForFirstMotor = stepsPerPixelX * xPixelNumber * 0.95;
    //        stepsForSecondMotor = stepsPerPixelY * yPixelNumber * 1.2;
    //
    //    }
    //
    //    if (
    //            (detectedObject.x < x_resolution / 4 * 1)
    //            &&
    //            (detectedObject.y < y_resolution / 4 * 4)
    //            &&
    //            (detectedObject.y > y_resolution / 4 * 3)
    //            ) {
    //        std::cout << "CELL 4 \n";
    //        stepsForFirstMotor = stepsPerPixelX * xPixelNumber * 0.87;
    //        stepsForSecondMotor = stepsPerPixelY * yPixelNumber * 0.93;
    //
    //    }
    //
    //    if (
    //            (detectedObject.x < x_resolution / 4 * 1)
    //            &&
    //            (detectedObject.y < y_resolution / 4 * 4)
    //            &&
    //            (detectedObject.y > y_resolution / 4 * 3)
    //            ) {
    //        std::cout << "CELL 4 \n";
    //        stepsForFirstMotor = stepsPerPixelX * xPixelNumber * 0.87;
    //        stepsForSecondMotor = stepsPerPixelY * yPixelNumber * 0.93;
    //
    //    }


    if (detectedObject.x > x_resolution / 2)
        stepsForFirstMotor = -stepsForFirstMotor;
    if (detectedObject.y > y_resolution / 2)
        stepsForSecondMotor = -stepsForSecondMotor;
    std::cout << "calculated steps using calibrated numbers " << stepsForFirstMotor << " " << stepsForSecondMotor << std::endl;

}

void PixelToMotorStepsConverter::calculateStepsUsingInverseKinematics(cv::Point& detectedObject, int& stepsForFirstMotor, int& stepsForSecondMotor) {

    //int bigZ = 10000;
    //int divider = 1;
    //// PERSPECTIVE PROJECTIONS
    //// POINT IN CAMERA FRAME. BUT WE NEED A POINT IN WORLD FRAME.
    //// 
    //// focal length is measured in pixels. x in pixels, bigZ in pixels
    //// 1cm == 500 px
    //bigZ = 10000 / divider;
    //std::cout << (detectedObject.x - x_resolution / 2) << " " << (detectedObject.y - y_resolution / 2) << "\n";
    //double bigX = (detectedObject.x - x_resolution / 2) * bigZ / focal_length;
    //double bigY = (detectedObject.y - y_resolution / 2) * bigZ / focal_length;
    //double angle1 = std::atan2((detectedObject.x - x_resolution / 2),focal_length) * 180/3.1415;
    //double angle2 = std::atan2((detectedObject.y - y_resolution / 2),focal_length) * 180/3.1415;
    //std::cout << angle1 << " " << angle2 << "\n";
    //InverseForwardKinematicsModel iks;



    //std::cout << "calculateStepsUsingInverseKinematics. \n " <<
    //        "Calculated coordinate of detected point:" << bigX << " " << bigY << " " << bigZ << "\n";
    ////
    ////    bigZ = 5000;
    ////    // focal length is measured in pixels. x in pixels, bigZ in pixels
    ////    bigX = detectedObject.x * bigZ / focal_length;
    ////    bigY = detectedObject.y * bigZ / focal_length;
    ////    std::cout << "Calculated coordinate of detected point:" << bigX << " " << bigY << " " << bigZ << "\n";



    //Eigen::Vector3d pointInCameraFrame = iks.convertCoordinateFromCameraToWorldFrame(Eigen::Vector3d(bigX, bigY, bigZ));


    //Eigen::Vector2d calculatedAngles = iks.doInverseKinematics(pointInCameraFrame);
    //std::cout << "calculated Angles using Perspective projection + Inverse kinematics: "
    //        << calculatedAngles[0] << " " << calculatedAngles[1] << "\n";


    //std::cout <<" atan ANGLES"<< angle1 << " " << angle2 << "\n";

    //double coef = 0.7;
    //stepsForFirstMotor = calculatedAngles[0] * stepsPer1AngleX * 2 * coef;
    //stepsForSecondMotor = calculatedAngles[1] * stepsPer1AngleY * 2 * coef;

    //std::cout << "calculated steps using calibrated numbers " << stepsForFirstMotor << " " << stepsForSecondMotor << std::endl;

}



