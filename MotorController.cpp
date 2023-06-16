#include "MotorController.h"
#include <wiringPi.h>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <unistd.h>        //Needed for I2C port
#include <fcntl.h>         //Needed for I2C port
#include <sys/ioctl.h>     //Needed for I2C port
#include <linux/i2c-dev.h> //Needed for I2C port
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include <fstream>
#include "ServerController.h"
#include <vector>
// axisBorders[0].left = -60;
// axisBorders[0].right = 300;
// axisBorders[0].home = 120;

// axisBorders[1].left = 167;
// axisBorders[1].right = -13;
// axisBorders[1].home = 77;

// axisBorders[0].left = 300;
// axisBorders[0].right = -60;
// axisBorders[0].home = 120;

// axisBorders[1].left = -13;
// axisBorders[1].right = 167;
// axisBorders[1].home = 77;

// homePositions[0] = 120;
// homePositions[1] = 77;
// axisBorders[1].cw = -3;
// axisBorders[1].ccw = 157;
// axisBorders[1].home = 77;

// axisBorders[0].cw = 250;
// axisBorders[0].ccw = -30;
// axisBorders[0].home = 120;

MotorController::~MotorController()
{
    std::cout << "~MotorController count" << inverseForwardKinematicsModel.use_count() << "\n";
    // delete inverseForwardKinematicsModel;
}
// updates kinematics angles
void MotorController::startAngleUpdatingOfKinematicsModelProc()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // cap.open(0);
    // if (!cap.isOpened()) {
    //     assert("ERROR! Unable to open camera\n");
    // }

    std::vector<int> angles = {0, 0};
    //  std::vector<int> v = {7, 5, 16, 8};
    for (;;)
    {

        angles.at(0) = currentAngle[0];
        angles.at(1) = currentAngle[1];
        inverseForwardKinematicsModel->updateCurrentThetasUsingCustomAngles(angles);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void MotorController::startAngleUpdatingOfKinematicsModel()
{
    std::thread thread_(&MotorController::startAngleUpdatingOfKinematicsModelProc, this);
    thread_.detach();
}

MotorController::MotorController(std::shared_ptr<InverseForwardKinematicsModel> &inverseForwardKinematicsModel_)
{

    inverseForwardKinematicsModel = inverseForwardKinematicsModel_;
    std::cout << "MotorController count" << inverseForwardKinematicsModel.use_count() << "\n";
    savedReceivedAngle[0] = 999;
    savedReceivedAngle[1] = 999;
    isTheAutohomingRunning = false;
    memset(receivedIds.data(), -1, NUMBER_OF_ANGLES);
    memset(receivedAngle.data(), -1, NUMBER_OF_ANGLES);
    memset(textMsg.data(), '\0', MAX_IP_PACK_SIZE);
    currentAngleToRequest = 0;

    shouldInverseSignOfReceivedAngleFromClient[0] = true;
    shouldInverseSignOfReceivedAngleFromClient[1] = false;
    isThereSomePersonControllingAxis = false;
    currentMovementID[0] = 0;
    currentMovementID[1] = 0;

    stepPins[0] = 17;
    stepPins[1] = 23;
    // stepPins[2] = 22;
    // stepPins[3] = 19;
    // stepPins[4] = 6;
    // stepPins[5] = 20;

    dirPins[0] = 18;
    dirPins[1] = 24;
    // dirPins[2] = 27;
    // dirPins[3] = 26;
    // dirPins[4] = 13;
    // dirPins[5] = 21;

    pinMode(dirPins[1], OUTPUT);
    pinMode(stepPins[1], OUTPUT);
    pinMode(dirPins[0], OUTPUT);
    pinMode(stepPins[0], OUTPUT);

    digitalWrite(dirPins[1], LOW);
    digitalWrite(dirPins[0], LOW);

    int speedIncreaser = 1;
    int speedDecreaser1 = 1;

    int speedDecreaser2 = 0.4;

    isNegativeCurrentAngle[0] = false;
    isNegativeCurrentAngle[1] = false;

    microseconds[0] = 50000 / speedIncreaser * speedDecreaser1;
    microseconds[1] = 50000 / speedIncreaser * speedDecreaser2;
    inc[0] = 1;
    inc[1] = 1;

    steps[0] = 100 * inc[0];
    steps[1] = 100 * inc[1];

    speedDecreaser1 = 3;
    speedDecreaser2 = 3;
    // 1 sec = 1000 ms = 1 000 000 us/
    // here every 0.15 sec rotation happens = 6 or 7 times per sec
    // 150000 is fast
    autohomingMicroseconds[0] = 50000 / speedIncreaser * speedDecreaser1;
    autohomingMicroseconds[1] = 50000 / speedIncreaser * speedDecreaser2;

    int autohomingVal = 10;
    autohomingSteps[0] = autohomingVal;
    autohomingSteps[1] = autohomingVal;

    homeWasReached[0] = false;
    homeWasReached[1] = false;

    forwardValue[0] = true;  // ccw
    forwardValue[1] = false; // ccw

    // 1 300ccw;120;300cw //14
    // 2 347ccw;77;167//30

    // axisBorders[0].left = -60;
    // axisBorders[0].right = 300;
    // axisBorders[0].home = 120;

    // axisBorders[1].left = -13;
    // axisBorders[1].right = 167;
    // axisBorders[1].home = 77;

    // axisBorders[1].cw = -3;
    // axisBorders[1].ccw = 157;
    // axisBorders[1].home = 77;

    // axisBorders[0].cw = 250;
    // axisBorders[0].ccw = -30;
    // axisBorders[0].home = 120;
    axisBorders[0].cw = 250;
    axisBorders[0].ccw = -30;
    axisBorders[0].home = 120;

    axisBorders[1].cw = -3;
    axisBorders[1].ccw = 157;
    axisBorders[1].home = 77;

    homePositions[0] = 120;
    homePositions[1] = 77;

    isCcwIncreasesValueOfMagnetEncoder[0] = false; // ccw decreases angle val from magnet
    isCcwIncreasesValueOfMagnetEncoder[1] = true;  //+

    isThereMovementToSpecificAngle[0] = false;
    isThereMovementToSpecificAngle[1] = false;

    startRotatyEncoders();

    isTheAutohomingRunning = false;
    // startAutohoming();
    startAngleUpdatingOfKinematicsModel();
}

// axisBorders[0].cw = 250;
// axisBorders[0].ccw = -30;
// axisBorders[0].home = 120;

// axisBorders[1].cw = -3;
// axisBorders[1].ccw = 157;
// axisBorders[1].home = 77;
bool MotorController::isValidAngle(int angleToReach, int currentAxis)
{
    if ((axisBorders[currentAxis].cw < axisBorders[currentAxis].ccw) &&
        (angleToReach < axisBorders[currentAxis].cw || angleToReach > axisBorders[currentAxis].ccw))
    {
        std::cout << currentAxis << " axis. out of the range. wrong angle ==" << angleToReach << "\n";
        return false;
    }
    else if ((axisBorders[currentAxis].cw > axisBorders[currentAxis].ccw) &&
             (angleToReach > axisBorders[currentAxis].cw || angleToReach < axisBorders[currentAxis].ccw))
    {
        std::cout << currentAxis << " axis. out of the range. wrong angle ==" << angleToReach << "\n";
        return false;
    }
    return true;
}

void MotorController::turretMoveAxesToSomeAngle(bool isRelativeMovement, std::vector<int> &anglesToReach, bool isTCPclient)
{
    if (!isThereMovement())
    {
        moveAxisToSomeAngleI(0, isRelativeMovement, anglesToReach[0], isTCPclient);
        moveAxisToSomeAngleI(1, isRelativeMovement, anglesToReach[1], isTCPclient);
    }
}

void MotorController::moveAxisToSomeAngleI(int currentAxis, bool isRelativeMovement, int angleToReach, bool isTCPclient)
{
    int calculatedAngleToReach = 0;
    calculatedAngleToReach = calculateAbsoluteOrRealtiveAngles(isRelativeMovement, currentAxis, angleToReach);

    std::cout << "calculatedAngleToReach= " << calculatedAngleToReach << " axis =" << currentAxis << "\n";

    if (isThereMovementToSpecificAngle[currentAxis])
        return;

    isThereMovementToSpecificAngle[currentAxis] = true;

    if (isTCPclient && !isValidAngle(calculatedAngleToReach, currentAxis))
    {
        isThereMovementToSpecificAngle[currentAxis] = false;
        ++personControllingAxis[currentAxis]->room_.name_to_id[currentAxis][personControllingAxis[currentAxis]->nicknameStr];
        std::cout << "person=" << personControllingAxis[currentAxis]->nicknameStr << ", id = " << personControllingAxis[currentAxis]->room_.name_to_id[currentAxis][personControllingAxis[currentAxis]->nicknameStr] << std::endl;
        return;
    }

    if (!isValidAngle(calculatedAngleToReach, currentAxis))
    {
        return;
    }

    if (currentAxis == 0)
    {
        moveAxisToSomeAngle1(calculatedAngleToReach, isTCPclient);
    }
    else if (currentAxis == 1)
    {
        moveAxisToSomeAngle2(calculatedAngleToReach, isTCPclient);
    }
}

// axisBorders[0].left = -60;
// axisBorders[0].right = 300;
// axisBorders[0].home = 120;

// axisBorders[1].left = -13;
// axisBorders[1].right = 167;
// axisBorders[1].home = 77;
void MotorController::moveAxisToSomeAngle1(int angleToReach, bool isTCPclient)
{
    int currentAxis = 0;

    // std::cout << "111 \n";
    // stop current movement if it exists
    // if (isThereMovementToSpecificAngle[currentAxis] == true)
    //     isThereMovementToSpecificAngle[currentAxis] = false;
    // sleep 100ms
    // usleep(100000);

    auto lambdaMoveAxisToSpecificAngle = [angleToReach, isTCPclient, this]()
    {
        moveAxisToSpecificAngle1(angleToReach, isTCPclient);
    };

    std::thread threadMoveToSpecificAngle(lambdaMoveAxisToSpecificAngle);
    threadMoveToSpecificAngle.detach();
}

void MotorController::moveAxisToSomeAngle2(int angleToReach, bool isTCPclient)
{
    int currentAxis = 1;
    // std::cout << "111_2 \n";
    // stop current movement if it exists
    // if (isThereMovementToSpecificAngle[currentAxis] == true)
    //     isThereMovementToSpecificAngle[currentAxis] = false;
    // sleep 100ms
    // usleep(100000);
    auto lambdaMoveAxisToSpecificAngle = [angleToReach, isTCPclient, this]()
    {
        moveAxisToSpecificAngle2(angleToReach, isTCPclient);
    };
    std::thread threadMoveToSpecificAngle(lambdaMoveAxisToSpecificAngle);
    threadMoveToSpecificAngle.detach();
}

int MotorController::getSpeedtimeForAccelerationZone(int i, int stepsNum)
{
    int timeOfslowestSpeed = 450000;

    float startRad = 1.57f;

    float curRad = 1.57f;
    float endRad = 0.33f;
    float radInc = (curRad - endRad) / stepsNum;

    curRad = startRad - radInc * i;
    if (curRad < endRad)
        curRad = endRad;

    int returnSpeedTime = timeOfslowestSpeed * sin(curRad);

    std::cout << "getAccel" << i << " " << curRad << " " << sin(curRad) << " " << returnSpeedTime << std::endl;

    return returnSpeedTime;
}
    // int timeOfslowestSpeed = 450000;

    // float startRad = 1.57f;

    // float curRad = 1.57f;
    // float endRad = 0.33f;
    // float radInc = (curRad - endRad) / stepsNum;


    // curRad = startRad - radInc * i;
    // if (curRad < endRad) curRad = endRad;

    // int returnSpeedTime = timeOfslowestSpeed * sin(curRad);

    // std::cout << i << " " << curRad << " " << sin(curRad) << " " << returnSpeedTime << std::endl;

    // return returnSpeedTime;
int MotorController::getSpeedtimeForDecelerationZone(int i, int stepsNum)
{
    int timeOfslowestSpeed = 450000;

    float endRad = 1.57f;
    float startRad = 0.33f;
    float curRad = 0;

    float radInc = (endRad - curRad) / stepsNum;

    curRad = startRad + radInc * i;
    if (curRad > endRad)
        curRad = endRad;

    int returnSpeedTime = timeOfslowestSpeed * sin(curRad);

    std::cout << "getDec" << i << " " << curRad << " " << sin(curRad) << " " << returnSpeedTime << std::endl;

    return returnSpeedTime;
}

void MotorController::moveAxisToSpecificAngle1(int angleToReach, bool isTCPclient)
{
    // std::cout << "222 \n";
    int currentAxis = 0;

    // was 90 need 120.
    // #1 deceleration for the last 10 degrees. acceleration for the first 10 degrees.
    // #2 deceleration for the last 30 %. acceleration for the first 30 %.

    // #2 wins a cookie. So, how to do it?
    // what parameters do we have to manipulate with?
    // digitalWrite(dirPins[axisIndex], forwardValue[axisIndex]);
    // move_motor(stepPins[axisIndex], microseconds, autohomingSteps[axisIndex]); // high = ccw
    // microseconds = time between shaft rotation
    // autohomingSteps[axisIndex] = little amount of step to move

    // void MotorController::move_motor2(int led, unsigned int time, int numberOfSteps)
    // {

    //     for (int i = 0; i < numberOfSteps; i++)
    //     {
    //         digitalWrite(led, HIGH);
    //         //    usleep(time);
    //         std::this_thread::sleep_for(std::chrono::nanoseconds(time));
    //         digitalWrite(led, LOW);
    //         //    usleep(time);
    //         std::this_thread::sleep_for(std::chrono::nanoseconds(time));
    //     }
    // }
    // than closer to the end=lesser degrees left to go, then time becomes bigger = movement slower.
    // $ have some function = sin cos.

    // max speed is 150k us. min speed is 40k. starting from 40k we accelerate to 150k and vise versa reaching the end
    //  for (float ph = 0; ph < 1.57f; ph += 0.05f) {
    //      float pos_m0 = 150000 * sin(ph);
    //      //float pos_m1 = 20000.0f * sin(ph);
    //      std::cout << ph << " " << sin(ph) << " " << pos_m0 << std::endl;
    //  }
    int minimalAngle = 4;
    int difference = abs(angleToReach - currentAngle[currentAxis]);
    int currentMicrosec = 0;
    int maxSpeedTime = 150000;
    int minSpeedTime = 450000;
    int borderPercentage = 0.3;
    int thirtyPercentsOfAngles = difference * borderPercentage;
    int savedCurrentAngle = currentAngle[currentAxis];
    int i = 0;
    int i2 = 0;
    int firstSwitchAngle;
    int secondSwitchAngle;
    class Zones
    {
    public:
        int accelerationZone[2];
        int decelerationZone[2];
    };
    Zones z;

    if (currentAngle[currentAxis] < angleToReach)
    {
        firstSwitchAngle = currentAngle[currentAxis] + difference * borderPercentage;
        secondSwitchAngle = currentAngle[currentAxis] + difference * borderPercentage * 2;
        z.accelerationZone[0] = currentAngle[currentAxis];
        z.accelerationZone[1] = currentAngle[currentAxis] + difference * borderPercentage;

        z.decelerationZone[0] = currentAngle[currentAxis] + difference * borderPercentage * 2;
        z.decelerationZone[1] = angleToReach;
    }
    else if (currentAngle[currentAxis] > angleToReach)
    {
        firstSwitchAngle = currentAngle[currentAxis] - difference * borderPercentage;
        secondSwitchAngle = currentAngle[currentAxis] - difference * borderPercentage * 2;

        // from small to big
        z.accelerationZone[0] = currentAngle[currentAxis] - difference * borderPercentage;
        z.accelerationZone[1] = currentAngle[currentAxis];

        z.decelerationZone[0] = angleToReach;
        z.decelerationZone[1] = currentAngle[currentAxis] - difference * borderPercentage * 2;
    }

    isThereMovementToSpecificAngle[currentAxis] = true;
int numberOfStepsInOneDegree = 5000;
    while ((currentAngle[currentAxis] != angleToReach) && isThereMovementToSpecificAngle[currentAxis])
    {
        if (difference >= minimalAngle)
        {
            if (currentAngle[currentAxis] > z.accelerationZone[0] && currentAngle[currentAxis] < z.accelerationZone[1])
            {
                currentMicrosec = getSpeedtimeForAccelerationZone(i, abs(z.accelerationZone[1]-z.accelerationZone[0])*numberOfStepsInOneDegree);
                i++;
                std::cout << "accelZone \n";
            }
            else if (currentAngle[currentAxis] > z.decelerationZone[0] && currentAngle[currentAxis] < z.decelerationZone[1])
            {
                currentMicrosec = getSpeedtimeForDecelerationZone(i2, abs(z.accelerationZone[1]-z.accelerationZone[0])*numberOfStepsInOneDegree);
                i2++;
                std::cout << "decZone \n";
            }
            else
            {
                currentMicrosec = maxSpeedTime;
            }
        }
        else
        {
            currentMicrosec = minSpeedTime;
        }

        int currentMicrosec = 0;
        if (currentAngle[currentAxis] < angleToReach)
        {
            if (isCcwIncreasesValueOfMagnetEncoder[currentAxis])
            {
                moveAxisCcw1(currentAxis, currentMicrosec);
            }
            else
            {
                moveAxisCw1(currentAxis, currentMicrosec);
            }
        }
        else if (currentAngle[currentAxis] > angleToReach)
        {
            if (isCcwIncreasesValueOfMagnetEncoder[currentAxis])
            {
                moveAxisCw1(currentAxis, currentMicrosec);
            }
            else
            {
                moveAxisCcw1(currentAxis, currentMicrosec);
            }
        }
    }

    isThereMovementToSpecificAngle[currentAxis] = false;

    if (isTCPclient)
    {
        ++personControllingAxis[currentAxis]->room_.name_to_id[currentAxis][personControllingAxis[currentAxis]->nicknameStr];
        std::cout << "cur id0 = " << personControllingAxis[currentAxis]->room_.name_to_id[currentAxis][personControllingAxis[currentAxis]->nicknameStr] << std::endl;
    }
}

void MotorController::moveAxisToSpecificAngle2(int angleToReach, bool isTCPclient)
{
    // std::cout << "222 2\n";
    int currentAxis = 1;
    // if (angleToReach < axisBorders[currentAxis].cw || angleToReach > axisBorders[currentAxis].ccw)
    // {
    //     std::cout << currentAxis << " axis reached Axis border \n";
    //     isThereMovementToSpecificAngle[currentAxis] = false;
    //     ++currentMovementID[currentAxis];
    //     ++personControllingAxis[currentAxis]->room_.name_to_id[currentAxis][personControllingAxis[currentAxis]->nicknameStr];
    //     std::cout << "cur id1 = " << personControllingAxis[currentAxis]->room_.name_to_id[currentAxis][personControllingAxis[currentAxis]->nicknameStr] << std::endl;

    //     return;
    // }

    // std::cout << "333 2\n";
    isThereMovementToSpecificAngle[currentAxis] = true;
    while ((currentAngle[currentAxis] != angleToReach) && isThereMovementToSpecificAngle[currentAxis])
    {
        if (currentAngle[currentAxis] < angleToReach)
        {
            // static int counter = 0;
            // counter++;
            if (isCcwIncreasesValueOfMagnetEncoder[currentAxis])
            {
                // if(counter == 1000)
                // std::cout<< " movingforward1 2nd axis \n";
                moveAxisCcw2(currentAxis, microseconds[currentAxis]);
            }
            else
            {
                // if(counter == 1000)
                // std::cout<< " movingback1 2nd axis \n";
                moveAxisCw2(currentAxis, microseconds[currentAxis]);
            }
            // if(counter == 1000) counter = 0;
        }
        else if (currentAngle[currentAxis] > angleToReach)
        {
            if (isCcwIncreasesValueOfMagnetEncoder[currentAxis])
            {
                // std::cout<< " movingback2 2nd axis \n";
                moveAxisCw2(currentAxis, microseconds[currentAxis]);
            }
            else
            {
                // std::cout<< " movingforward2 2nd axis \n";
                moveAxisCcw2(currentAxis, microseconds[currentAxis]);
            }
        }
    }
    isThereMovementToSpecificAngle[currentAxis] = false;
    ++currentMovementID[currentAxis];
    if (isTCPclient)
    {
        ++personControllingAxis[currentAxis]->room_.name_to_id[currentAxis][personControllingAxis[currentAxis]->nicknameStr];

        std::cout << "cur id1 = " << personControllingAxis[currentAxis]->room_.name_to_id[currentAxis][personControllingAxis[currentAxis]->nicknameStr] << std::endl;
    }
}

bool MotorController::checkIfRobotIsAtHome()
{
    int numberAxesAtHome = 0;

    for (int i = 0; i < NUMBER_OF_AXES; i++)
    {
        if (currentAngle[i] == homePositions[i])
        {
            ++numberAxesAtHome;
        }
    }

    if (numberAxesAtHome == NUMBER_OF_AXES)
    {
        isTheAutohomingRunning = false;
        std::cout << "bot is at home \n";
    }

    return numberAxesAtHome == NUMBER_OF_AXES;
}

void MotorController::moveAxisCcw1(int axisIndex, int microseconds)
{
    digitalWrite(dirPins[axisIndex], forwardValue[axisIndex]);
    move_motor(stepPins[axisIndex], microseconds, autohomingSteps[axisIndex]); // high = ccw
}

void MotorController::moveAxisCw1(int axisIndex, int microseconds)
{
    digitalWrite(dirPins[axisIndex], !forwardValue[axisIndex]);
    move_motor(stepPins[axisIndex], microseconds, autohomingSteps[axisIndex]);
}

void MotorController::moveAxisCcw2(int axisIndex, int microseconds)
{
    digitalWrite(dirPins[axisIndex], forwardValue[axisIndex]);
    move_motor2(stepPins[axisIndex], microseconds, autohomingSteps[axisIndex]); // high = ccw
}

void MotorController::moveAxisCw2(int axisIndex, int microseconds)
{
    digitalWrite(dirPins[axisIndex], !forwardValue[axisIndex]);
    move_motor2(stepPins[axisIndex], microseconds, autohomingSteps[axisIndex]);
}

// Blink an LED
void MotorController::move_motor(int led, unsigned int time, int numberOfSteps)
{

    for (int i = 0; i < numberOfSteps; i++)
    {
        digitalWrite(led, HIGH);
        //    usleep(time);
        std::this_thread::sleep_for(std::chrono::nanoseconds(time));
        digitalWrite(led, LOW);
        //    usleep(time);
        std::this_thread::sleep_for(std::chrono::nanoseconds(time));
    }
}

// Blink an LED
void MotorController::move_motor2(int led, unsigned int time, int numberOfSteps)
{

    for (int i = 0; i < numberOfSteps; i++)
    {
        digitalWrite(led, HIGH);
        //    usleep(time);
        std::this_thread::sleep_for(std::chrono::nanoseconds(time));
        digitalWrite(led, LOW);
        //    usleep(time);
        std::this_thread::sleep_for(std::chrono::nanoseconds(time));
    }
}

// axisBorders[0].cw = 250;
// axisBorders[0].ccw = -30;
// axisBorders[0].home = 120;

// axisBorders[1].cw = -3;
// axisBorders[1].ccw = 157;
// axisBorders[1].home = 77;

// isCcwIncreasesValueOfMagnetEncoder[0] = false; // ccw decreases angle val from magnet
// isCcwIncreasesValueOfMagnetEncoder[1] = true;  //+

int MotorController::calculateAbsoluteOrRealtiveAngles(bool isRelativeMovement, int axis, int angleToReach)
{
    int calculatedAngleToReach = 0;
    if (isRelativeMovement)
    {

        // 0 was 150
        // 1 was 50

        // ccw 0 decreases = false
        // ccw 1 increases = true

        // received 10 10

        // must become 140 60. because positive value is ccw in math model. so we move axes 10 degree around Z ccw.
        // need to move = 0ccw 10, 1ccw 10
        // need to reach = 140, 60
        //
        if (!isCcwIncreasesValueOfMagnetEncoder[axis])
        {
            calculatedAngleToReach = currentAngle[axis] - angleToReach;
            // motorController_->moveAxisToSomeAngleI(motorController_->currentAngle[axis] - motorController_->savedReceivedAngle[axis], axis);
        }
        else
        {
            calculatedAngleToReach = currentAngle[axis] + angleToReach;
            // motorController_->moveAxisToSomeAngleI(motorController_->currentAngle[axis] + motorController_->savedReceivedAngle[axis], axis);
        }
    }
    else
    {
        // 0 home is 120. ccw decreases val of sensor
        // 1 home is 77. ccw inc.
        // value 10 and 10 of math model.
        // need to reach 110 and 87.
        if (!isCcwIncreasesValueOfMagnetEncoder[axis])
        {
            calculatedAngleToReach = axisBorders[axis].home - angleToReach;
            // motorController_->moveAxisToSomeAngleI(motorController_->axisBorders[axis].home - motorController_->savedReceivedAngle[axis], axis);
        }
        else
        {
            calculatedAngleToReach = axisBorders[axis].home + angleToReach;
            // motorController_->moveAxisToSomeAngleI(motorController_->axisBorders[axis].home + motorController_->savedReceivedAngle[axis], axis);
        }
    }
    return calculatedAngleToReach;
}

void MotorController::readVal(uint16_t *buf)
{
    if (read(file_i2c, buf, 1) != 1) // read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
    {
        // ERROR HANDLING: i2c transaction failed
        printf("Failed to read from the i2c bus.\n");
    }
    else
    {
        //    printf("read val: %ld\n", buf[0]);
    }
}

void MotorController::writeVal(uint16_t valToWrite)
{
    //  uint16_t buffer[1];
    //  uint16_t addres[1];

    if (write(file_i2c, &valToWrite, 1) != 1) // write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
    {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n");
    }
}

void MotorController::writeVal(uint16_t address, uint16_t valToWrite)
{
    writeVal(address);
    writeVal(valToWrite);
}

void MotorController::readAngle()
{

    int length;
    uint16_t lowval = 0;
    uint16_t highval = 0;
    uint16_t buffer[1];
    uint16_t buffer2[1];

    ///////////////////////////////////read low val
    buffer[0] = 0x0D;

    writeVal(buffer[0]);

    buffer[0] = 0;

    //----- READ BYTES -----
    length = 1; //<<< Number of bytes to read

    readVal(&lowval);

    {
        //    printf("Data read1: %ld\n", lowval);
    }

    //  for (uint8_t i = 0; i < length; i++) {
    //    printf("read val = %d \n", buffer[i]);
    //  }

    /////////////////////////////////////read high val

    // read status
    buffer[0] = 0x0C;

    length = 1;

    writeVal(buffer[0]);

    readVal(&highval);

    {
        highval <<= 8;
    }
    // 1 300b;330;330 //30
    // 2 38f;52;66  //14
    // 3 292f;316;340 //24
    // 4 95;95;125b
    // 5 354;8;22f
    // 6 344;8;32f

    // 1 167ccw;77;347 //30
    // 2 300ccw;120;300cw //14
    if (currentAngleToRequest == 0)
    {
        //    currentAngle[1] = (lowval | highval) * 0.087890625;
        // sensors return only from 0 to 360
        //
        int curAngle = 1;
        int newVal = (lowval | highval) * 0.087890625;
        axisBorders[1].cw = -3;

        if (newVal > 360)
            return;
        if (currentAngle[curAngle] >= 0 && currentAngle[curAngle] <= 10 && newVal > 300)
        {
            // goes to negative side = from 0 to -1, -10, -30 etc
            isNegativeCurrentAngle[curAngle] = true;
        }
        // goes to positive side = from -1 to 1, 10, 30 etc
        else if (isNegativeCurrentAngle[curAngle] && currentAngle[curAngle] < 0 && newVal >= 0 && newVal <= 60)
        {
            isNegativeCurrentAngle[curAngle] = false;
        }

        if (isNegativeCurrentAngle[curAngle] == true)
        {

            // got 300, was 1, 300-360==-60 == current value
            currentAngle[curAngle] = newVal - 360;
        }
        else
        {
            currentAngle[curAngle] = newVal;
        }

        //      currentAngle[1] = newVal - 360;
        //    else
        //      currentAngle[1] = newVal;

        //    currentAngle[5] = (lowval | highval) * 0.087890625;
    }
    else if (currentAngleToRequest == 1)
    {
        int curAngle = 0;
        int newVal = (lowval | highval) * 0.087890625;
        if (currentAngle[curAngle] >= 0 && currentAngle[curAngle] <= 10 && newVal > 300)
        {
            isNegativeCurrentAngle[curAngle] = true;
        }
        else if (isNegativeCurrentAngle[curAngle] && currentAngle[curAngle] < 0 && newVal >= 0 && newVal <= 60)
        {
            isNegativeCurrentAngle[curAngle] = false;
        }

        if (isNegativeCurrentAngle[curAngle] == true)
        {
            currentAngle[curAngle] = newVal - 360;
        }
        else
        {
            currentAngle[curAngle] = newVal;
        }
    }
    //  else if (currentAngleToRequest == 2)
    //  {
    //    currentAngle[0] = (lowval | highval) * 0.087890625;
    //  }
    //  else if (currentAngleToRequest == 3)
    //  {
    //    currentAngle[3] = (lowval | highval) * 0.087890625;
    //  }
    //  else if (currentAngleToRequest == 4)
    //  {
    //    int newVal = (lowval | highval) * 0.087890625;
    //    if (newVal > 330)
    //      currentAngle[4] = newVal - 360;
    //    else
    //      currentAngle[4] = newVal;
    //
    ////    currentAngle[4] = (lowval | highval) * 0.087890625;
    //  }
    //  else if (currentAngleToRequest == 5)
    //  {
    //    int newVal = (lowval | highval) * 0.087890625;
    //    if (newVal > 330)
    //      currentAngle[5] = newVal - 360;
    //    else
    //      currentAngle[5] = newVal;
    //
    ////    currentAngle[5] = (lowval | highval) * 0.087890625;
    //  }
}

void MotorController::readStatusAngle()
{
    int length;
    uint16_t buffer[1];
    uint16_t buffer2[1];

    buffer2[0] = 1;
    /////////////////////////////////////read status
    buffer[0] = 0x0B; //   00001011

    writeVal(buffer[0]);

    buffer[0] = 0;
    //----- READ BYTES -----
    length = 1; //<<< Number of bytes to read
    readVal(buffer);
    {
        //    printf("status: %ld\n", buffer[0]);
    }

    if ((buffer[0] & 32) == 32)
    {
        readAngle();
    }
    else
    {
        //    printf("not ((buffer[0] & 32) == 32) \n");
    }
}

void MotorController::startRotatyEncoders()
{
    string filename = "/dev/i2c-1";
    if ((file_i2c = open(filename.c_str(), O_RDWR)) < 0)
    {
        // ERROR HANDLING: you can check errno to see what went wrong
        printf("Failed to open the i2c bus");
        return;
    }

    auto readAngle1 = [this]()
    {
        while (true)
        {
            int MUX_address = 0x70; //<<<<<The I2C address of the slave

            if (ioctl(file_i2c, I2C_SLAVE, MUX_address) < 0)
            {
                printf("Failed to acquire bus access and/or talk to slave.\n");
                return -1;
            }

            writeVal(MUX_address, 1 << currentAngleToRequest);

            const int as5600_address = 0x36;
            if (ioctl(file_i2c, I2C_SLAVE, as5600_address) < 0)
            {
                printf("Fail to reach laser \n");
                exit(EXIT_FAILURE);
            }

            readStatusAngle();
            if (currentAngleToRequest == NUMBER_OF_AXES - 1)
                currentAngleToRequest = 0;
            else
                ++currentAngleToRequest;

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };

    auto displayAngles = [this]()
    {
        unsigned int microsecond1 = 1500000;
        while (true)
        {
            std::cout << "angle1=" << currentAngle[0] << ", angle2=" << currentAngle[1] << " \n";
            // printf("angle1=%ld, angle2=%ld\n", currentAngle[0], currentAngle[1]);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    };

    std::thread threadReadAngle1(readAngle1);
    std::thread threadDisplayAngles(displayAngles);

    threadReadAngle1.detach();
    threadDisplayAngles.detach();
}

// axisBorders[0].left = -60;
// axisBorders[0].right = 300;
// axisBorders[0].home = 120;

// axisBorders[1].left = -13;
// axisBorders[1].right = 167;
// axisBorders[1].home = 77;
// as result of these functions 1st axis ll have 300 and 2nd axis ll have 167.
void MotorController::reachCcwLimit1()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int angleBefore = 0;
    int angleAfter = 0;
    int currentAxis = 0;
    int counter = 0;
    int counterEnd = 10;

    // axisBorders[0].cw = 250;
    // axisBorders[0].ccw = -30;
    // axisBorders[0].home = 120;

    // axisBorders[1].cw = -3;
    // axisBorders[1].ccw = 157;
    // axisBorders[1].home = 77;

    while (true)
    {

        if (counter == 0)
            angleBefore = currentAngle[currentAxis];

        // both axes have false, so axes ll go ccw in both cases
        // when an angle stops its movement it ll change a sign of new value of encoders

        moveAxisCcw1(currentAxis, autohomingMicroseconds[currentAxis]);

        if (counter == counterEnd)
        {

            angleAfter = currentAngle[currentAxis];

            if (angleBefore == angleAfter)
            {
                // isNegativeCurrentAngle[currentAxis] = true;
                // if (currentAngle[currentAxis] > 0) {
                //     currentAn
                std::cout << "0 AXIS reached limit \n";
                // }
                // currentAngle[0] = currentAngle[0] - 360;
                isNegativeCurrentAngle[0] = true;
                return;
            }

            counter = 0;
        }
        else
        {
            counter++;
        }
    }
}

void MotorController::reachCcwLimit2()
{

    // auto t_start = std::chrono::high_resolution_clock::now();
    // // the work...
    // auto t_end = std::chrono::high_resolution_clock::now();
    // double elapsed_time_ms = 300;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int angleBefore = 0;
    int angleAfter = 0;
    int currentAxis = 1;
    int counter = 0;
    int counterEnd = 70;
    while (true)
    {

        if (counter == 0)
            angleBefore = currentAngle[currentAxis];

        // if (!isCcwIncreasesValueOfMagnetEncoder[currentAxis])
        // {
        moveAxisCcw2(currentAxis, autohomingMicroseconds[currentAxis]);
        // }
        // else
        // {
        //     moveAxisCw2(currentAxis, autohomingMicroseconds[currentAxis]);
        // }
        if (counter == counterEnd)
        {

            angleAfter = currentAngle[currentAxis];

            if (angleBefore == angleAfter)
            {
                // isNegativeCurrentAngle[currentAxis] = true;

                std::cout << "1 AXIS reached limit \n";
                return;
            }
            counter = 0;
        }
        else
        {
            counter++;
        }
    }
}

bool MotorController::isThereMovement()
{

    for (int i = 0; i < NUMBER_OF_AXES; i++)
    {
        if (isThereMovementToSpecificAngle[i])
        {
            std::cout << "there is some movement of axis = " << i << "\n";
            return true;
        }
    }

    return false;
}

void MotorController::startAutohoming()
{
    if (checkIfRobotIsAtHome())
        return;
    // std::thread threadAutohoming0(autohoming0);
    // std::thread threadAutohoming1(autohoming1);
    // threadAutohoming0.join();
    // threadAutohoming1.join();
    isTheAutohomingRunning = true;
    std::thread threadReachLimit1(&MotorController::reachCcwLimit1, this);

    std::thread threadReachLimit2(&MotorController::reachCcwLimit2, this);

    threadReachLimit1.join();

    threadReachLimit2.join();

    auto autohoming0 = [this]()
    {
        //      void MotorController::moveAxisForward1(int axisIndex)
        // {
        //     digitalWrite(dirPins[axisIndex], forwardValue[axisIndex]);
        //     move_motor(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); // high = back
        // }
        //   while (true)
        //   {
        // gotoCcwLimit()
        //
        // moveAxis0ForwardUntilAngleStopedChanging();
        // isNegativeCurrentAngle[curAngle] = true;
        // goHomeFunction

        // axisBorders[1].left = -13;

        // if (void DidAngleStopChanging();
        // axisBorders[1].left = -13;
        // axisBorders[1].right = 167;
        // axisBorders[1].home = 77;

        // axisBorders[0].left = -60;
        // axisBorders[0].right = 300;
        // axisBorders[0].home = 120;
        // homePositions[0] = 120;
        // homePositions[1] = 77;
        int currentAxis = 0;
        while (currentAngle[currentAxis] != homePositions[currentAxis])
        {
            if (currentAngle[currentAxis] < homePositions[currentAxis])
            {
                if (isCcwIncreasesValueOfMagnetEncoder[currentAxis])
                {
                    moveAxisCcw1(currentAxis, autohomingMicroseconds[currentAxis]);
                }
                else
                {
                    moveAxisCw1(currentAxis, autohomingMicroseconds[currentAxis]);
                }
            }
            else if (currentAngle[currentAxis] > homePositions[currentAxis])
            {
                if (isCcwIncreasesValueOfMagnetEncoder[currentAxis])
                {
                    moveAxisCw1(currentAxis, autohomingMicroseconds[currentAxis]);
                }
                else
                {
                    moveAxisCcw1(currentAxis, autohomingMicroseconds[currentAxis]);
                }
            }
        }
        if (checkIfRobotIsAtHome())
            isTheAutohomingRunning = false;
        std::cout << "0 AXIS is AT HOME \n";
    };

    auto autohoming1 = [this]()
    {
        int currentAxis = 1;
        while (currentAngle[currentAxis] != homePositions[currentAxis])
        {
            if (currentAngle[currentAxis] < homePositions[currentAxis])
            {
                if (isCcwIncreasesValueOfMagnetEncoder[currentAxis])
                {
                    moveAxisCcw2(currentAxis, autohomingMicroseconds[currentAxis]);
                }
                else
                {
                    moveAxisCw2(currentAxis, autohomingMicroseconds[currentAxis]);
                }
            }
            else if (currentAngle[currentAxis] > homePositions[currentAxis])
            {
                if (isCcwIncreasesValueOfMagnetEncoder[currentAxis])
                {
                    moveAxisCw2(currentAxis, autohomingMicroseconds[currentAxis]);
                }
                else
                {
                    moveAxisCcw2(currentAxis, autohomingMicroseconds[currentAxis]);
                }
            }
        }

        if (checkIfRobotIsAtHome())
            isTheAutohomingRunning = false;

        std::cout << "1 AXIS is AT HOME \n";
    };

    std::thread threadAutohoming0(autohoming0);
    std::thread threadAutohoming1(autohoming1);
    threadAutohoming0.join();
    threadAutohoming1.join();
    isTheAutohomingRunning = false;
}
