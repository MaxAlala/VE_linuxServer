#include "ServerController.h"
#include "MotorController.h"
#include "RobotSystem.h"
#include <wiringPi.h>

int main(int argc, char *argv[])
{
    wiringPiSetupGpio();
    RobotSystem rs;
    rs.startRobotSystem();

    // for(int i = 0; i < 20; i++)
    // serverController.motorController.moveAxisCcw1(0, 50000);
}
