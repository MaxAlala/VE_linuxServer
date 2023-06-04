#pragma once
#include "VoiceController.h"
#include "MotorController.h"
#include "ServerController.h"
class RobotSystem {
public:
RobotSystem();
void startRobotSystem();
std::unique_ptr<MotorController> motorController;
std::unique_ptr<VoiceController> voiceController;
std::unique_ptr<ServerController> serverController;
};

