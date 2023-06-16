#pragma once
#include "VoiceController.h"
#include "MotorController.h"
#include "ServerController.h"
#include "Eye.h"
#include "InverseForwardKinematicsModel.h"
#include "PixelToMotorStepsConverter.h"
#include "WebServer.h"
#include <string>
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
void startWebServer();
// void bringToLife();
void startFaceDetectionForOneSec();
void startLidarDistanceDetectionProc();
void startLidarDistanceDetection();

void startUpdateTimeOfLifeEveryMinuteThread();
void startUpdateTimeOfLifeEveryMinute();
void sendStartingTime();
int timeOfbeingOnline;
std::string timeOfStartUTC; 
std::shared_ptr<MotorController> motorController;
std::shared_ptr<VoiceController> voiceController;
std::shared_ptr<ServerController> serverController;
std::shared_ptr<Eye> visionController;
std::shared_ptr<InverseForwardKinematicsModel> inverseForwardKinematicsModel;
std::shared_ptr<PixelToMotorStepsConverter> pixelToMotorStepsConverter;
// std::shared_ptr<WebServer> ws;
drogon::orm::DbClientPtr db;
bool isVisionOn = false;
uint16_t currentLidarDistance = 0;
int turretID = 1;

static void setIsPatrolOn(bool isPatrolOnToSet) {
    isPatrolOn = isPatrolOnToSet;
}

private:
  States currentState;
  CurrentRoboticSystem currentRoboticSystem;
  static int isPatrolOn;
  bool wasPatrolThreadStarted = false;
};

