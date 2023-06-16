/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   InverseForwardKinematicsModel.cpp
 * Author: sol
 * 
 * Created on January 4, 2021, 9:52 AM
 */

#include "InverseForwardKinematicsModel.h"
#include <string>
#include <iostream>
#include <sstream>
#include <cmath>
#include "protocol.hpp"
#include <mutex>
// #define M_PI           3.1415  /* pi */

std::mutex mu;
void shared_cout(std::string msg)
{
    std::lock_guard<std::mutex> guard(mu);
    std::cout << msg << std::endl;
}

const int InverseForwardKinematicsModel::NUMBER_OF_MATH_AXES;
// init position == directed up
InverseForwardKinematicsModel::InverseForwardKinematicsModel() :
kinematicAxisNumber{4},
// -90 if 
// ---
// |
// | 
thetasWhichCanBeChanged{1,1,0,0},
currentThetas{180, 0, 90, 0},
// 0 if
//  |
//  |
//  |
default_thetas{180, 0, 90, 0},
offset_d{link_length[0], 0, link_length[2], link_length[3]},
link_length_a{0, 0, 0, 0},
alphas{-90, 90, -90, 0},
link_length{35, 0, 10, 10},
DH{kinematicAxisNumber},
numOfPixelsInOneCM{1},
inverseKinematicsCalculatedAngles{0,0,0,0},
previousAngles{0,0,0,0},
pseudoCurrentActionAngles{0,0,0,0},
pseudoPreviousActionAngles{0,0,0,0},
currentThetasWhereStartIsZero{0,0,0,0}
{
    //fromGradToRad(default_thetas);
    //fromGradToRad(alphas);
    //    thetas[1] += fromGradToRad(90);
    for (int i = 0; i < kinematicAxisNumber; i++) {
        DH_parameters(i, 0) = fromGradToRad(default_thetas.at(i));
        DH_parameters(i, 1) = offset_d.at(i)*numOfPixelsInOneCM;
        DH_parameters(i, 2) = link_length_a.at(i)*numOfPixelsInOneCM;
        DH_parameters(i, 3) = fromGradToRad(alphas.at(i));
    }
}

InverseForwardKinematicsModel::InverseForwardKinematicsModel(const InverseForwardKinematicsModel& orig) {
}

InverseForwardKinematicsModel::~InverseForwardKinematicsModel() {
    std::cout << "~InverseForwardKinematicsModel(); \n";
}

int InverseForwardKinematicsModel::getCurrentActionAngle(int angleNum) {
    return inverseKinematicsCalculatedAngles.at(angleNum);
}

void InverseForwardKinematicsModel::destroyPseudoZeros(Eigen::Matrix<double, 3, 3 > &mat) {
    for (int i = 0; i < mat.rows(); i++)
        for (int j = 0; j < mat.cols(); j++)
            if (mat(i, j) < 0.00001)
                if (mat(i, j) > -0.00001)
                    mat(i, j) = 0.0;
}

void InverseForwardKinematicsModel::truncate(double& val, int numDigits) {
    using namespace std;
    std::string output = std::to_string(val).substr(0, numDigits + 1);

    // case where . at the beginning or at the end of the game
    if (output.find('.') == string::npos ||
            output.back() == '.') {
        output.pop_back();
    }
    std::stringstream ss;
    ss << output;
    ss >> val;
}

void InverseForwardKinematicsModel::checkIfAlmostEqual(double &x, double &y) {
    std::cout.precision(17);
    std::cout << "x: " << x << ", y: " << y << std::endl;
    std::cout << "abs(x - y): " << abs(abs(x) - abs(y)) << std::endl;
    if (abs(abs(x) - abs(y)) < 0.000001) {
        bool isDifferentSignes = false; // false == same signes
        if (x >= 0 && y <= 0) {
            x = -y;
            isDifferentSignes = true;
        }
        if (y >= 0 && x <= 0) {
            y = -x;
            isDifferentSignes = true;
        }
        std::cout << "Yes! checkIfAlmostEqual:  \n";

    }

}

double InverseForwardKinematicsModel::fromGradToRad(double d) {
    return d / 180.0 * M_PI;
}

double InverseForwardKinematicsModel::fromRadToGrad(double d) {
    return d * 180.0 / M_PI;
}

void InverseForwardKinematicsModel::fromGradToRad(std::vector<int> &vec) {
    std::for_each(vec.begin(), vec.end(), [](int &d) {
        d = d / 180 * M_PI; });

}

void InverseForwardKinematicsModel::printThetas(std::vector<int >& thetas) {
    for (int i = 0; i < kinematicAxisNumber; i++) {
        std::string str2 = "theta i = " + std::to_string(i) + " = " + std::to_string(thetas.at(i)) + " \n";
        shared_cout(str2);
    }
}

void InverseForwardKinematicsModel::printActionAngles() {
    for (int i = 0; i < kinematicAxisNumber; i++) {
        std::string str2 = "currentActionAngles i = " + std::to_string(i) + " = " + std::to_string(inverseKinematicsCalculatedAngles.at(i)) + " \n";
        shared_cout(str2);
    }

    for (int i = 0; i < kinematicAxisNumber; i++) {
        std::string str2 = "previousActionAngles i = " + std::to_string(i) + " = " + std::to_string(previousAngles.at(i)) + " \n";
        shared_cout(str2);
    }
}

void InverseForwardKinematicsModel::printStepAngles() {
    double stepAngle1 = 0;
    double stepAngle2 = 0;

    std::string str2 = "currentActionAngles = " + std::to_string(inverseKinematicsCalculatedAngles.at(0)) + " " + std::to_string(inverseKinematicsCalculatedAngles.at(1));
    shared_cout(str2);

    str2 = "previousActionAngles = " + std::to_string(previousAngles.at(0)) + " " + std::to_string(previousAngles.at(1));
    shared_cout(str2);

    // CREATE ANGLES TO MOVE WITH RESPECT CURRENT ANGLES
    if (previousAngles.at(0) == 0) {
        stepAngle1 = inverseKinematicsCalculatedAngles.at(0);
    }
    else {
        stepAngle1 = inverseKinematicsCalculatedAngles.at(0) - previousAngles.at(0);
    }

    if (previousAngles.at(1) == 0) {
        stepAngle2 = inverseKinematicsCalculatedAngles.at(1);
    }
    else {
        stepAngle2 = inverseKinematicsCalculatedAngles.at(1) - previousAngles.at(1);
    }

    std::string str = "STEP ANGLES = " + std::to_string(stepAngle1) + " " + std::to_string(stepAngle2);
    shared_cout(str);

}

Eigen::Matrix<double, 4, 4 > InverseForwardKinematicsModel::getDH4_0() {
    return DH_4_0;
}

void InverseForwardKinematicsModel::calculateDH(std::vector<int >& thetas) {
    for (int i = 0; i < kinematicAxisNumber; i++) {
        DH_parameters(i, 0) = fromGradToRad(thetas.at(i));
        DH_parameters(i, 1) = offset_d.at(i)*numOfPixelsInOneCM;
        DH_parameters(i, 2) = link_length_a.at(i)*numOfPixelsInOneCM;
        DH_parameters(i, 3) = fromGradToRad(alphas.at(i));
    }

    if (DH.size() > 0)DH.clear();
    
    for (int i = 0; i < kinematicAxisNumber; i++) {
        Eigen::Matrix<double, 4, 4> current_from_i_to_i_plus_one_mat;
        current_from_i_to_i_plus_one_mat << cos(DH_parameters(i, 0)), -sin(DH_parameters(i, 0)) * cos(DH_parameters(i, 3)), sin(DH_parameters(i, 0)) * sin(DH_parameters(i, 3)), DH_parameters(i, 2) * cos(DH_parameters(i, 0)),
                sin(DH_parameters(i, 0)), cos(DH_parameters(i, 0)) * cos(DH_parameters(i, 3)), -cos(DH_parameters(i, 0)) * sin(DH_parameters(i, 3)), DH_parameters(i, 2) * sin(DH_parameters(i, 0)),
                0, sin(DH_parameters(i, 3)), cos(DH_parameters(i, 3)), DH_parameters(i, 1),
                0, 0, 0, 1;
        DH.push_back(Eigen::Matrix<double, 4, 4>(current_from_i_to_i_plus_one_mat));
    }


    DH_2_0 = DH[0] * DH[1];
    // std::cout << "DH1_0 \n" << DH[0] << "\n";

    // std::cout << "DH2_1 \n" << DH[1] << "\n";

    // std::cout << "DH3_2 \n" << DH[2] << "\n";

    // std::cout << "DH4_3 \n" << DH[3] << "\n";

    DH_4_0 = DH[0] * DH[1] * DH[2] * DH[3];
    // std::cout << " DH_4_0 == \n " << DH_4_0 << std::endl;


    // camera coordinate system = DH3_2 in plate coordinate system DH2_1
    //Eigen::Matrix<double, 4, 4> DH3_2;

    //DH3_2 <<
    //        0,  0, 1, 9 * numOfPixelsInOneCM,
    //       -1,  0, 0, 0,
    //        0, -1, 0, 9 * numOfPixelsInOneCM,
    //        0, 0, 0, 1;

    //
    //DH.push_back(DH3_2);


 /*   DH.at(0) <<
        -1, 0, 0, 0,
        0, 0, -1, 0,
        0, -1, 0, 35,
        0, 0, 0, 1;

    DH.at(1) <<
        1, 0, 0, 0,
        0, 0, -1, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;

    DH.at(2) <<
        0, 0, -1, -5,
        1, 0, 0, 0,
        0, -1, 0, 5,
        0, 0, 0, 1;*/

}

void InverseForwardKinematicsModel::setDefaultThetas(std::vector<int >& thetas) {
    for (int i = 0; i < kinematicAxisNumber; i++) {
        thetas.at(i) = default_thetas[i];
    }
}

void InverseForwardKinematicsModel::setDefaultActionAngles() {
    for (int i = 0; i < kinematicAxisNumber; i++) {
        inverseKinematicsCalculatedAngles.at(i) = 0;
        previousAngles.at(i) = 0;
    }
}

void InverseForwardKinematicsModel::setDefaultPseudoActionAngles() {
    for (int i = 0; i < kinematicAxisNumber; i++) {
        pseudoCurrentActionAngles.at(i) = 0;
        pseudoPreviousActionAngles.at(i) = 0;
    }
}
/** @brief Updates thetas so that DH for forward kinematics corresponded to real current position of turret.
* default thetas + calculatedAngles =
 *
 *  @param shouldIgnore1Angle
 *  @param shouldIgnore2Angle
 *  @pre
 *  @post in brief
 *  @return void
 */
void InverseForwardKinematicsModel::updateThetasUsingAnglesFromInverseKinematics(std::vector<int >& thetas, bool shouldIgnore1Angle, bool shouldIgnore2Angle)
{
    // THETAS UPDATE
    // firstly, set default home position thetas.
    setDefaultThetas(thetas);
    
    // secondly, adds action angles to thetas so now DH represents real position + rotation
    for (int i = 0; i < kinematicAxisNumber; i++) {

        // calculatedTheta == absolute angle which goes from beginning position
        //if (!((shouldIgnore1Angle && (i == 0)) || (shouldIgnore2Angle && (i == 1)))) {
        //    std::cout << i << " new theta = " << inverseKinematicsCalculatedAngles[i] + thetas[i] << "\n";
        //    thetas[i] = inverseKinematicsCalculatedAngles[i] + thetas[i]; // or we could just assign calculatedTheta
        //}
        thetas[i] = inverseKinematicsCalculatedAngles[i]; // or we could just assign calculatedTheta
    }
}



void InverseForwardKinematicsModel::updateCurrentThetasUsingCustomAngles(std::vector<int>& angles)
{
    class AxisBorders
    {
    public:
        int left;
        int right;
        int home;
    };

    AxisBorders axisBorders[2];
    // //default_thetas{180, 0, 90, 0},
    axisBorders[1].left = -13;
    axisBorders[1].right = 167;
    axisBorders[1].home = 77;

    axisBorders[0].left = -60;
    axisBorders[0].right = 300;
    axisBorders[0].home = 120;

    // THETAS UPDATE
    // firstly, set default home position thetas.
    //setDefaultThetas();
    //std::cout << "updateThetasUsingCustomAngles \n";

    int differenceFromHomePosition[4];
    differenceFromHomePosition[0] = 0;
    differenceFromHomePosition[1] = 0;
    differenceFromHomePosition[2] = 0;
    differenceFromHomePosition[3] = 0;
    // // 120 - -60 = 180? must be 60.
    // //
    for(int i = 0; i < 2; i++) {
        differenceFromHomePosition[i] = axisBorders[i].home - angles.at(i);
    }

    // secondly, adds action angles to thetas so now DH represents real position + rotation
    for (int i = 0; i < kinematicAxisNumber; i++) {

        // calculatedTheta == absolute angle which goes from beginning position
        if (thetasWhichCanBeChanged[i]) {
            //std::cout << angles.at(i) << "=received angle, i = " << i << std::endl;
            //std::cout << i << " new custom theta = " << differenceFromHomePosition[i] + default_thetas[i] << "\n";
            currentThetasWhereStartIsZero[i] = differenceFromHomePosition[i];
            currentThetas[i] = differenceFromHomePosition[i] + default_thetas[i]; // or we could just assign calculatedTheta
        }
    }
}

/** @brief Updates thetas so that DH for forward kinematics was correct.
 *
 *  @param shouldIgnore1Angle
 *  @param shouldIgnore2Angle
 *  @pre
 *  @post in brief
 *  @return void
 */
void InverseForwardKinematicsModel::updateThetasUsingPseudoCurrentAngles(bool shouldIgnore1Angle, bool shouldIgnore2Angle)
{
    // THETAS UPDATE
    // firstly, set default home position thetas.
    setDefaultThetas(currentThetas);

    // secondly, adds action angles to thetas so now DH represents real position + rotation
    for (int i = 0; i < kinematicAxisNumber; i++) {

        // ignores some angle updates
        if (!((shouldIgnore1Angle && (i == 0)) || (shouldIgnore2Angle && (i == 1)))) {

            // calculatedTheta == absolute angle which goes from beginning position
            std::cout << i << " pseudo theta = " << pseudoCurrentActionAngles[i] + fromRadToGrad(currentThetas[i]) << "\n";
            currentThetas[i] = fromGradToRad(pseudoCurrentActionAngles[i] + fromRadToGrad(currentThetas[i])); // or we could just assign calculatedTheta
        }
    }
}

std::vector<int >& InverseForwardKinematicsModel::getCurrentThetas() {
    return currentThetas;
}

std::vector<int >& InverseForwardKinematicsModel::getCalculationThetas() {
    return calculationThetas;
}

/** @brief  1) Here uses angles from inverse kinematics to create stepAngles = how much degrees need to rotate from
 *          current position.
 *          to calculate step angle u should subtract old angle from new angle. E.g. u old angle was 45, new angle is -45, to reach -45 from 45 u should move -90 (-45 - 45).
 *  @pre
 *  @post in brief
 *  @return stepAngles
 */
std::vector<int> InverseForwardKinematicsModel::calculateRelativeMovementAngles()
{
    int stepAngle1 = 0;
    int stepAngle2 = 0;
    previousAngles = currentThetasWhereStartIsZero;
    //std::string str2 = "inverseKinematicsCalculatedAngles = " + std::to_string(inverseKinematicsCalculatedAngles.at(0)) + " " + std::to_string(inverseKinematicsCalculatedAngles.at(1));
    //shared_cout(str2);
    //for (int i = 0; i < 2; i++) {
    //    previousAngles.at(i) = fromRadToGrad(previousAngles.at(i));
    //}

    //str2 = "previousActionAngles = " + std::to_string(previousAngles.at(0)) + " " + std::to_string(previousAngles.at(1));
    //shared_cout(str2);

    // CREATE ANGLES TO MOVE WITH RESPECT CURRENT ANGLES

    stepAngle1 = inverseKinematicsCalculatedAngles.at(0) - previousAngles.at(0);
    stepAngle2 = inverseKinematicsCalculatedAngles.at(1) - previousAngles.at(1);

    std::string str = "previous ANGLES = " + std::to_string(previousAngles.at(0)) + " " + std::to_string(previousAngles.at(1));
    shared_cout(str);
    str = "inverseK ANGLES = " + std::to_string(inverseKinematicsCalculatedAngles.at(0)) + " " + std::to_string(inverseKinematicsCalculatedAngles.at(1));
    shared_cout(str);
    str = "STEP ANGLES = " + std::to_string(stepAngle1) + " " + std::to_string(stepAngle2); 
    shared_cout(str);

    std::vector<int> angles;
    angles.push_back(stepAngle1);
    angles.push_back(stepAngle2);

    return angles;
}

void InverseForwardKinematicsModel::doExtendedInverseKinematics(Eigen::Vector3d P0) {

    //Eigen::Vector3d P0;
    //P0 << 100, 100, 100;

    // here thetas were updated = Y2_0 was placed perpendicular to the plane of P0 vector
    // so now possible to rotate around Y2_0 to get new coordinate

    // calculated pseudo angles to move y2_0 to the same plane as p0
    doInverseKinematics(currentThetas, P0, false, true);

    // updated thetas to create new DH = only 1 theta = Y2_0 was placed perpendicular to the plane of P0 vector
    // updates only pseudo thetas = just to calculate a new point using H2_0, rotated P0 around Y2_0 by 90 degrees.
    updateThetasUsingPseudoCurrentAngles(false, true);

    // creates new DH. receives a new H2_0 with rotated 0 axis 
    calculateDH(currentThetas);

    Eigen::Matrix<double, 4, 4 > DH_2_0; // from 2 to 0

    // calculates DH2_0
    DH_2_0 = DH[0] * DH[1];
    std::cout << "DH1_0 \n" << DH[0] << "\n";

    std::cout << "DH2_1 \n" << DH[1] << "\n";

    std::cout << "DH3_2 \n" << DH[2] << "\n";

    Eigen::Matrix<double, 4, 4 > DH_3_0; // from 2 to 0
    DH_3_0 = DH[0] * DH[1] * DH[2];

    std::cout << " DH_3_0 == \n " << DH_3_0 << std::endl;

    // CONVERTS P0 IN P2 TO ROTATE P2 around Y2_0
    // u cannot rotate P0 without expressing it into DH2_0, expressed point is P2
    Eigen::Matrix<double, 3, 3 > R_2_0;
    R_2_0 <<
        DH_2_0(0, 0), DH_2_0(0, 1), DH_2_0(0, 2),
        DH_2_0(1, 0), DH_2_0(1, 1), DH_2_0(1, 2),
        DH_2_0(2, 0), DH_2_0(2, 1), DH_2_0(2, 2);


    Eigen::Matrix<double, 3, 3 > R_0_2;
    R_0_2 = R_2_0.transpose();

    std::cout << " R_0_2 == \n " << R_0_2 << std::endl;


    Eigen::Matrix<double, 3, 3 > R_0_2_negative;
    R_0_2_negative = -R_0_2;

    Eigen::Vector3d P0_2;
    P0_2 << DH_2_0(0, 3), DH_2_0(1, 3), DH_2_0(2, 3);
    std::cout << " P2_0 == \n " << P0_2 << std::endl;
    
    // this is how origin of 0 expressed in 2 coordinate system
    P0_2 = R_0_2_negative * P0_2;

    std::cout << " P0_2 == \n " << P0_2 << std::endl;

    Eigen::Matrix <double, 4, 4 > DH_0_2;

    // 0 coordinate system expressed in 2
    DH_0_2 <<
        R_0_2(0, 0), R_0_2(0, 1), R_0_2(0, 2), P0_2(0),
        R_0_2(1, 0), R_0_2(1, 1), R_0_2(1, 2), P0_2(1),
        R_0_2(2, 0), R_0_2(2, 1), R_0_2(2, 2), P0_2(2),
        0, 0, 0, 1;

    std::cout << " DH_0_2 == \n " << DH_0_2 << std::endl;
    std::cout << " DH_2_0 == \n " << DH_2_0 << std::endl;

    // HERE P0 expressed in 2 COORD. SYST.
    Eigen::Vector3d P2;
    P2 << R_0_2 * P0 + P0_2;
    std::cout << " P2 == \n " << P2 << std::endl;

    // rotation around y2 by 90 degrees
    Eigen::Matrix <double, 3, 3 > R_rot_around_Y_2_0;
    R_rot_around_Y_2_0 <<
        cos(M_PI / 2), 0, sin(M_PI / 2),
        0, 1, 0,
        -sin(M_PI / 2), 0, cos(M_PI / 2);


    std::cout << "R_rot_around_Y_2_0 \n" << R_rot_around_Y_2_0 << "\n";

    Eigen::Vector3d P2_afterY2_0Rot;

    // HERE ROTATED P2
    P2_afterY2_0Rot << R_rot_around_Y_2_0 * P2;

    std::cout << "P2 \n" << P2 << "\n";


    Eigen::Vector3d P0_afterY2_0Rot;
    Eigen::Vector3d P2_0;

    P2_0 << DH_2_0(0, 3), DH_2_0(1, 3), DH_2_0(2, 3);

    // HERE ROTATED P0
    P0_afterY2_0Rot = R_2_0 * P2_afterY2_0Rot + P2_0;

    std::cout << "P2_afterY2_0Rot \n" << P2_afterY2_0Rot << "\n";
    std::cout << "P0_afterY2_0Rot \n" << P0_afterY2_0Rot << "\n";

    setDefaultThetas(currentThetas);

    // calculates current angles for rotated point 
    doInverseKinematics(currentThetas, P0_afterY2_0Rot);

    // to remove ghost rotation = when y2 was placed into the same plane = no need now, cause have pseudo angles
    //setDefaultActionAngles();
    
    // updates thetas
    updateThetasUsingAnglesFromInverseKinematics(currentThetas);

    // now DH is rotated to the rotated point 
    calculateDH(currentThetas);

    std::cout << "DH1_0 \n" << DH[0] << "\n";

    std::cout << "DH2_1 \n" << DH[1] << "\n";

    std::cout << "DH3_2 \n" << DH[2] << "\n";

    DH_3_0 = DH[0] * DH[1] * DH[2];

    std::cout << " DH_3_0 == \n " << DH_3_0 << std::endl;

    //calculateStepAngles();

}

void InverseForwardKinematicsModel::doForwardKinematics(std::vector<int >& thetas) {

    calculateDH(thetas);

    Eigen::Matrix<double, 4, 4 > DH_2_0; // from 2 to 0
    DH_2_0 = DH[0] * DH[1];
    std::cout << "DH1_0 \n" << DH[0] << "\n";

    std::cout << "DH2_1 \n" << DH[1] << "\n";

    std::cout << "DH3_2 \n" << DH[2] << "\n";

    std::cout << "DH4_3 \n" << DH[3] << "\n";

    Eigen::Matrix<double, 4, 4 > DH_4_0; // from 2 to 0
    DH_4_0 = DH[0] * DH[1] * DH[2] * DH[3];
    std::cout << " DH_4_0 == \n " << DH_4_0 << std::endl;
}

Eigen::Vector3d InverseForwardKinematicsModel::convertCoordinateFromCameraToWorldFrame(std::vector<int >& thetas, Eigen::Vector3d pointWhereToGo) {

    Eigen::Vector4d pointInCam(pointWhereToGo[0], pointWhereToGo[1], pointWhereToGo[2], 1);
    Eigen::Vector4d pointInWorld;
    calculateDH(thetas);
    pointInWorld = DH[0] * DH[1] * DH[2] * pointInCam;

    std::cout << "point in cam frame: " << pointInCam << "\n";
    std::cout << "converted point from cam to world: " << pointInWorld << "\n";
    return Eigen::Vector3d(pointInWorld[0], pointInWorld[1], pointInWorld[2]);
}

/** @brief  Calculates angles to reach a given position from start position, puts them into currentActionAngles. Updates currentActionAngles & previousActionAngles
 *  @param pointWhereToGo
 *  @param shouldUpdateActionAngles
 *  @pre
 *  @post 
 *  @return updated currentActionAngles & previousActionAngles
 *                                  |
 *                                  |
 *                                  |
 */
void InverseForwardKinematicsModel::doInverseKinematics(std::vector<int >& thetas, Eigen::Vector3d pointWhereToGo, bool shouldUpdateActionAngles, bool shouldUpdatePseudoActionAngles) {
    //    Eigen::Vector3d pointWhereToGo{10, 10, 25};
    //double r1 = sqrt(pointWhereToGo[0] * pointWhereToGo[0] + pointWhereToGo[1] * pointWhereToGo[1]);
    //double r2 = pointWhereToGo[2] - link_length[0];

    //std::cout << "where to go:" << pointWhereToGo[0] << " " << pointWhereToGo[1] << " " << pointWhereToGo[2] << "\n";

    //std::cout << "r1 and r2: " << r1 << " " << r2 << "\n";
    //std::cout << "atan2(r2, r1) in grad ==" << fromRadToGrad(std::atan2(r2, r1)) << "\n";
    //std::cout << "atan2(r2, r1)) " << std::atan2(r2, r1) << "\n";

    //std::vector<double> calculatedTheta(axisNumber);
    //calculatedTheta[1] = -(90 - fromRadToGrad(std::atan2(r2, r1)));
    //calculatedTheta[0] = fromRadToGrad(std::atan2(pointWhereToGo[1], pointWhereToGo[0]));
    //std::cout << "doInverseKinematics. calculatedThetas = " << calculatedTheta[0] << " " << calculatedTheta[1] << std::endl;

    //////////////////////////////////////////
    calculateDH(default_thetas);
    
    Eigen::Vector3d secondCoordSystemOrigin = getCoordinateSystemOriginLocation(2);
    Eigen::Vector3d forthCoordSystemOrigin = getCoordinateSystemOriginLocation(4);

    // std::cout << "second origin = " << secondCoordSystemOrigin.x() << " " << secondCoordSystemOrigin.y() << " " << secondCoordSystemOrigin.z() << "\n";
    // std::cout << "forth origin = " << forthCoordSystemOrigin.x() << " " << forthCoordSystemOrigin.y() << " " << forthCoordSystemOrigin.z() << "\n";

    Eigen::Vector3d firstVector{
        forthCoordSystemOrigin.x() - secondCoordSystemOrigin.x(),
        forthCoordSystemOrigin.y() - secondCoordSystemOrigin.y(), 
        forthCoordSystemOrigin.z() - secondCoordSystemOrigin.z()
    };

    Eigen::Vector3d secondVector{
        pointWhereToGo.x() - secondCoordSystemOrigin.x(),
        pointWhereToGo.y() - secondCoordSystemOrigin.y(),
        pointWhereToGo.z() - secondCoordSystemOrigin.z()
    };


    Eigen::Vector3d thirdVector{
        pointWhereToGo.x() - forthCoordSystemOrigin.x(),
        pointWhereToGo.y() - forthCoordSystemOrigin.y(),
        pointWhereToGo.z() - forthCoordSystemOrigin.z()
    };

    // std::cout << "firstVector=" << firstVector.x() << " " << firstVector.y() << " " << firstVector.z() << std::endl;
    // std::cout << "secondVector=" << secondVector.x() << " " << secondVector.y() << " " << secondVector.z() << std::endl;
    // std::cout << "thirdVector=" << thirdVector.x() << " " << thirdVector.y() << " " << thirdVector.z() << std::endl;

    double magnitude1 =
        sqrt(
            firstVector.x() * firstVector.x() +
            firstVector.y() * firstVector.y() +
            firstVector.z() * firstVector.z()
    );

    double magnitude2 =
        sqrt(
            secondVector.x() * secondVector.x() +
            secondVector.y() * secondVector.y() +
            secondVector.z() * secondVector.z()
    );

    double magnitude3 =
        sqrt(
            thirdVector.x() * thirdVector.x() +
            thirdVector.y() * thirdVector.y() +
            thirdVector.z() * thirdVector.z()
        );

    double dotProduct =
        firstVector.x() * secondVector.x() +
        firstVector.y() * secondVector.y() +
        firstVector.z() * secondVector.z();

    // std::cout << "magnitude1=" << magnitude1 << "\n";
    // std::cout << "magnitude2=" << magnitude2 << "\n";
    // std::cout << "magnitude3=" << magnitude3 << "\n";
    // std::cout << "dotProduct=" << dotProduct << "\n";

    std::vector<double> calculatedTheta(kinematicAxisNumber);
    
    double r3 = sqrt(pointWhereToGo[0] * pointWhereToGo[0] + pointWhereToGo[1] * pointWhereToGo[1]);
    double r2 = pointWhereToGo[2] - link_length[0];
    double r1 = sqrt(link_length[2] * link_length[2] + link_length[3] * link_length[3]);

    double thi2_2 = acos(((magnitude3*magnitude3)-magnitude1*magnitude1-magnitude2*magnitude2)/-(2*magnitude1*magnitude2));
    // std::cout << "thi2_2=" << fromRadToGrad(thi2_2) << "\n";

    double acosval = acos((double)dotProduct / (double)(magnitude1 * magnitude2));
    double thi2 = fromRadToGrad(acosval);
    double thi4 = std::atan2(10 + magnitude3, 10);
    calculatedTheta[1] = (90 - fromRadToGrad(thi4) - fromRadToGrad(std::atan2(r2, r3)));

    // std::cout << "thi4=" << fromRadToGrad(thi4) << "\n";

    calculatedTheta[0] = fromRadToGrad(std::atan2(pointWhereToGo[1], pointWhereToGo[0]));


    // std::cout << "r3=" << r3 << "\n";
    // std::cout << "acosval=" << acosval << "\n";
    // std::cout << "thi2=" << thi2 << "\n";
    // std::cout << "where to go:" << pointWhereToGo[0] << " " << pointWhereToGo[1] << " " << pointWhereToGo[2] << "\n";
    // std::cout << "r1 and r2: " << r1 << " " << r2 << "\n";
    // std::cout << "thi3 = atan2(r2, r3) in grad ==" << fromRadToGrad(std::atan2(r2, r3)) << "\n";
    // std::cout << "atan2(r2, r3)) " << std::atan2(r2, r3) << "\n";
    // std::cout << "doInverseKinematics. calculatedThetas = " << calculatedTheta[0] << " " << calculatedTheta[1] << std::endl;
    
    std::vector<int> calculatedThetaInts(kinematicAxisNumber);
    inverseKinematicsCalculatedAngles.at(0) = (int)round(calculatedTheta[0]);
    inverseKinematicsCalculatedAngles.at(1) = (int)round(calculatedTheta[1]);

    //calculatedThetaInts.push_back((int)calculatedTheta[0]);
    //calculatedThetaInts.push_back((int)calculatedTheta[1]);
    std::cout << "doInverseKinematics. inverseKinematicsCalculatedAngles = " << inverseKinematicsCalculatedAngles[0] << " " << inverseKinematicsCalculatedAngles[1] << std::endl;

    //if (shouldUpdateActionAngles) 
    //   updateCalculatedAngles(calculatedThetaInts);

    if (shouldUpdatePseudoActionAngles) {
        pseudoPreviousActionAngles.at(0) = pseudoCurrentActionAngles.at(0);
        pseudoPreviousActionAngles.at(1) = pseudoCurrentActionAngles.at(1);
        pseudoCurrentActionAngles.at(0) = calculatedTheta[0];
        pseudoCurrentActionAngles.at(1) = calculatedTheta[1];
    }
}

std::vector<double> InverseForwardKinematicsModel::getInverseKinematicsAngles() {
    return inverseKinematicsCalculatedAngles;
}


Eigen::Vector3d InverseForwardKinematicsModel::getCoordinateSystemOriginLocation(int DhNum) {
    if (DhNum == 2)
        return Eigen::Vector3d{DH_2_0(0,3), DH_2_0(1,3), DH_2_0(2,3)};
    if (DhNum == 4)
        return Eigen::Vector3d{DH_4_0(0,3), DH_4_0(1,3), DH_4_0(2,3)};
    else return Eigen::Vector3d{0, 0, 0};
}

void InverseForwardKinematicsModel::updateCalculatedAngles(std::vector<int>& calculatedTheta) {
    //previousAngles.at(0) = calculatedAngles.at(0);
    //previousAngles.at(1) = calculatedAngles.at(1);
    inverseKinematicsCalculatedAngles.at(0) = calculatedTheta[0];
    inverseKinematicsCalculatedAngles.at(1) = calculatedTheta[1];
}

void InverseForwardKinematicsModel::updateThetasAddingNewAnglesToCurrentThetas(std::vector<int>& calculatedTheta)
{
    std::stringstream ss;
    std::string str;

    ss << "before theta[0]=" << fromRadToGrad(currentThetas[0]) << " " << "theta[1]=" << fromRadToGrad(currentThetas[1]) << std::endl;
    str = ss.str();
    ss.str("");
    shared_cout(str);
    currentThetas[0] += fromGradToRad(calculatedTheta[0]); // or we could just assign calculatedTheta
    currentThetas[1] += fromGradToRad(calculatedTheta[1]); // or we could just assign calculatedTheta
    
    ss << "after theta[0]=" << fromRadToGrad(currentThetas[0]) <<" "<< "theta[1]=" << fromRadToGrad(currentThetas[1]) << std::endl;
    str = ss.str();
    ss.str("");
    shared_cout(str);
}



