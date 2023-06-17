

class AccelerationDeceleration
{
    // 14 degree = 100 loops
    // 1 rev = 25600 steps
    //  1 loop = 10 steps
    // 100 loops = 1000 steps
    // 1 degree = 71 steps
    // 1000 steps/71 = 14 degree
    // all is right.
public:
    AccelerationDeceleration(int currentAngle_, int angleToReach_);
    void display();
    int generateMicrosec(int currentAngle);
    int getSpeedtimeForAccelerationZone(int i, int stepsNum);
    int getSpeedtimeForDecelerationZone(int i, int stepsNum);

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
};