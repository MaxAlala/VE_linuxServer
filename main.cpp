#include "ServerController.h"
#include "MotorController.h"
#include "RobotSystem.h"
#include <wiringPi.h>
#define BOOST_ERROR_CODE_HEADER_ONLY
#define BOOST_SYSTEM_NO_DEPRECATED
// #define USE_POSTGRESQL

void shared_cout(std::string msg);

int main(int argc, char *argv[])
{
    shared_cout(" STart1 \n");
    wiringPiSetupGpio();
    RobotSystem rs;
    shared_cout(" STart1.1 \n");
    rs.startRobotSystem();
    // shared_cout(" start2 \n");
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    shared_cout(" start3 \n");
    // for(int i = 0; i < 20; i++)
    // serverController.motorController.moveAxisCcw1(0, 50000);
    // system("pause");
    system("read -p 'Press Enter to continue...' var");
    return 0;
}









