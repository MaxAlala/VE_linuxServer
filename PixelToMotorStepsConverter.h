/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PixelToMotorStepsConverter.h
 * Author: sol
 *
 * Created on January 1, 2021, 8:03 PM
 */

#ifndef PIXELTOMOTORSTEPSCONVERTER_H
#define PIXELTOMOTORSTEPSCONVERTER_H
#include <vector>
#include <stdio.h>
#include "opencv2/core.hpp"
#include <vector>

class PixelToMotorStepsConverter {
public:
    PixelToMotorStepsConverter(int x_resolution, int y_resolution, int focal_length, int stepsPer1AngleX_, int stepsPer1AngleY_, double stepsPerPixelX = 0, double stepsPerPixelY = 0);
    PixelToMotorStepsConverter(const PixelToMotorStepsConverter& orig);
    virtual ~PixelToMotorStepsConverter();
    void calculateAnglesUsingLogic(cv::Point& detectedObject, int& angleX, int& angleY);
    void calculateStepsByPixel(cv::Point& detectedObject, int& stepsForFirstMotor, int& stepsForSecondMotor);
    void calculateStepsUsingCalibratedValue(cv::Point& detectedObject, int& stepsForFirstMotor, int& stepsForSecondMotor);
    void calculateStepsUsingInverseKinematics(cv::Point& detectedObject, int& stepsForFirstMotor, int& stepsForSecondMotor);
    
    double removeSign(double x);
    int fromRadToGrad(double x);
private:
    int angleX;
    int angleY;
    int x_resolution;
    int y_resolution;
    int focal_length;
    int stepsPer1AngleX;
    int stepsPer1AngleY;
    double stepsPerPixelX;
    double stepsPerPixelY;
};

#endif /* PIXELTOMOTORSTEPSCONVERTER_H */

