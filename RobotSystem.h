#pragma once
#include "VoiceController.h"
#include "MotorController.h"
#include "ServerController.h"
#include "Eye.h"
#include "InverseForwardKinematicsModel.h"
#include "PixelToMotorStepsConverter.h"

class RobotSystem {
    
    enum class States {
        ALIVE,
        MANUAL,
    };
public:
RobotSystem();
void startRobotSystem();
void startPatrol();
std::unique_ptr<MotorController> motorController;
std::unique_ptr<VoiceController> voiceController;
std::unique_ptr<ServerController> serverController;
std::unique_ptr<Eye> visionController;
std::unique_ptr<InverseForwardKinematicsModel> inverseForwardKinematicsModel;
std::unique_ptr<PixelToMotorStepsConverter> pixelToMotorStepsConverter;


private:
  States currentState;

};

