#include "AccelerationDeceleration.h"
AccelerationDeceleration::AccelerationDeceleration(int currentAngle_, int angleToReach_)
{
    initialCurrentAngle = currentAngle_;
    angleToReach = angleToReach_;
    difference = abs(angleToReach - initialCurrentAngle);
    difDivBy3 = difference / 3;
    numberOfStepsInOneDegree = 6;
    if (initialCurrentAngle < angleToReach)
    {
        accelerationZone[0] = initialCurrentAngle;
        accelerationZone[1] = initialCurrentAngle + difDivBy3;

        decelerationZone[0] = angleToReach - difDivBy3;
        decelerationZone[1] = angleToReach;
    }
    else if (initialCurrentAngle > angleToReach)
    {
        // from small to big
        accelerationZone[0] = initialCurrentAngle - difDivBy3;
        accelerationZone[1] = initialCurrentAngle;

        decelerationZone[0] = angleToReach;
        decelerationZone[1] = angleToReach + difDivBy3;
    }
}

int initialCurrentAngle;
int angleToReach;
int accelerationZone[2];
int decelerationZone[2];
int difDivBy3;
int minimalAngle = 4;
float difference;
int currentMicrosec = 0;
int maxSpeedTime = 150000;
int minSpeedTime = 450000;
int i = 0;
int i2 = 0;
int firstSwitchAngle;
int secondSwitchAngle;
int numberOfStepsInOneDegree;
int numberOfLoop = 0;

void AccelerationDeceleration::display()
{
    std::cout << "difference= " << difference << ", z.accelerationZone[0]=" << accelerationZone[0] << ", z.accelerationZone[1]=" << accelerationZone[1] << "\n";
    std::cout << "z.decelerationZone[0]=" << decelerationZone[0] << ", z.decelerationZone[1]=" << decelerationZone[1] << "\n";
}

int AccelerationDeceleration::generateMicrosec(int currentAngle)
{
    if (difference >= minimalAngle)
    {
        if (currentAngle >= accelerationZone[0] && currentAngle <= accelerationZone[1])
        {
            currentMicrosec = getSpeedtimeForAccelerationZone(i, abs(accelerationZone[1] - accelerationZone[0]) * numberOfStepsInOneDegree);
            i++;
            // std::cout << "accelZone \n";
        }
        else if (currentAngle >= decelerationZone[0] && currentAngle <= decelerationZone[1])
        {
            currentMicrosec = getSpeedtimeForDecelerationZone(i2, abs(decelerationZone[1] - decelerationZone[0]) * numberOfStepsInOneDegree);
            i2++;
            // std::cout << "decZone \n";
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
}

int AccelerationDeceleration::getSpeedtimeForAccelerationZone(int i, int stepsNum)
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
int AccelerationDeceleration::getSpeedtimeForDecelerationZone(int i, int stepsNum)
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
