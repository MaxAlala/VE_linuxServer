/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   InverseForwardKinematicsModel.h
 * Author: sol
 *
 * Created on January 4, 2021, 9:52 AM
 */

#ifndef INVERSEFORWARDKINEMATICSMODEL_H
#define INVERSEFORWARDKINEMATICSMODEL_H
#include <vector>
#include "./Eigen/Dense"


class InverseForwardKinematicsModel {
public:
    InverseForwardKinematicsModel();
    InverseForwardKinematicsModel(const InverseForwardKinematicsModel& orig);
    void doForwardKinematics(std::vector<int >& thetas);
    void doInverseKinematics(std::vector<int >& thetas, Eigen::Vector3d v, bool shouldUpdateActionAngles = true, bool shouldUpdatePseudoActionAngles = false);
    void doExtendedInverseKinematics(Eigen::Vector3d v);
    void calculateDH(std::vector<int >& thetas);
    void truncate(double& val, int numDigits);
    void checkIfAlmostEqual(double &x, double &y);
    double fromGradToRad(double d);
    void fromGradToRad(std::vector<int> &vec);
    double fromRadToGrad(double d);
    void destroyPseudoZeros(Eigen::Matrix<double, 3, 3 > &mat);
    void convertCoordinate();
    void setDefaultThetas(std::vector<int >& thetas);
    void setDefaultActionAngles();
    void setDefaultPseudoActionAngles();
    std::vector<int >& getCurrentThetas();
    std::vector<int >& getCalculationThetas();
    void updateThetasUsingAnglesFromInverseKinematics(std::vector<int >& thetas, bool shouldIgnore1Angle = false, bool shouldIgnore2Angle = false);
    void updateCurrentThetasUsingCustomAngles(std::vector<int>& angles);
    void updateThetasUsingPseudoCurrentAngles(bool shouldIgnore1Angle = false, bool shouldIgnore2Angle = false);
    void printThetas(std::vector<int >& thetas);
    void printActionAngles();
    void printStepAngles();
    Eigen::Matrix<double, 4, 4 > getDH4_0();
    std::vector<int> calculateRelativeMovementAngles();
    void updateCalculatedAngles(std::vector<int>& calculatedTheta);
    Eigen::Vector3d convertCoordinateFromCameraToWorldFrame(std::vector<int >& thetas, Eigen::Vector3d pointWhereToGo);
    void updateThetasAddingNewAnglesToCurrentThetas(std::vector<int>& calculatedTheta);
    Eigen::Vector3d getCoordinateSystemOriginLocation(int DhNum);
    virtual ~InverseForwardKinematicsModel();

    int getCurrentActionAngle(int angleNum);

    int kinematicAxisNumber;
    std::vector<Eigen::Matrix <double, 4, 4 >> DH;

    std::vector<int> getThetas() {
        return currentThetas;
    }
    std::vector<int > getThetasWhereStartIsZero() {
        return currentThetasWhereStartIsZero;
    }
    std::vector<double> getInverseKinematicsAngles();


    const static int NUMBER_OF_MATH_AXES = 4;
        std::vector<int > currentThetasWhereStartIsZero;
        std::vector<double > inverseKinematicsCalculatedAngles;
private:
    std::vector<double > link_length;

    //Updates thetas so that DH for forward kinematics corresponded to real current position of turret.
    std::vector<int > currentThetas;
    std::vector<int > calculationThetas;

    std::vector<bool > thetasWhichCanBeChanged;

    // 

    std::vector<int > previousAngles;
    std::vector<int > pseudoCurrentActionAngles;
    std::vector<int > pseudoPreviousActionAngles;
    std::vector<int > default_thetas;
    std::vector<double > offset_d;
    std::vector<double > link_length_a;
    std::vector<int > alphas;
    Eigen::Matrix<double, 4, 4 > DH_4_0; // from 4 to 0
    Eigen::Matrix<double, 4, 4 > DH_parameters;
    Eigen::Matrix<double, 4, 4 > DH_3_0; // from 2 to 0
    Eigen::Matrix<double, 4, 4 > DH_2_0; // from 2 to 0
    int numOfPixelsInOneCM; 


};

#endif /* INVERSEFORWARDKINEMATICSMODEL_H */

