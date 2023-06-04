#include "RobotSystem.h"
#include "tcpServer.h"
RobotSystem::RobotSystem() {


}

void RobotSystem::startRobotSystem() {
    ServerController serverController;

// std::unique_ptr<MotorController> motorController;
// std::unique_ptr<VoiceController> voiceController;
// std::unique_ptr<ServerController> serverController;

    motorController.reset(new MotorController());
    voiceController.reset(new VoiceController());
    serverController.reset(new ServerController());
    serverController->startServer();
}