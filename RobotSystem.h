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

        enum class CurrentRoboticSystem {
        TURRET,
        ANDROID,
    };

public:
RobotSystem();
~RobotSystem();
void startLifeFunc();
void startLife();
void startServer();
void startRobotSystem();
// void bringToLife();
void startFaceDetectionForOneSec();
void startLidarDistanceDetectionProc();
void startLidarDistanceDetection();
std::shared_ptr<MotorController> motorController;
std::shared_ptr<VoiceController> voiceController;
std::shared_ptr<ServerController> serverController;
std::shared_ptr<Eye> visionController;
std::shared_ptr<InverseForwardKinematicsModel> inverseForwardKinematicsModel;
std::shared_ptr<PixelToMotorStepsConverter> pixelToMotorStepsConverter;
uint16_t currentLidarDistance = 0;

private:
  States currentState;
  CurrentRoboticSystem currentRoboticSystem;
};

