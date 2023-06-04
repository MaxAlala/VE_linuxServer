#include "ServerController.h"
#include "MotorController.h"
#include "RobotSystem.h"
int main(int argc, char *argv[])
{
    wiringPiSetupGpio();
    RobotSystem rs;
    rs.startServer();

    // for(int i = 0; i < 20; i++)
    // serverController.motorController.moveAxisCcw1(0, 50000);
}
