#include "ServerController.h"
#include "MotorController.h"
#include "RobotSystem.h"
#include <wiringPi.h>

void shared_cout(std::string msg);

int main(int argc, char *argv[])
{

        shared_cout(" STart1 \n");
    wiringPiSetupGpio();
    RobotSystem rs;
    shared_cout(" STart1.1 \n");
    // rs.startRobotSystem();
    // shared_cout(" start2 \n");
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    shared_cout(" start3 \n");
    // for(int i = 0; i < 20; i++)
    // serverController.motorController.moveAxisCcw1(0, 50000);
}
