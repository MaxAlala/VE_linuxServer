#pragma once


#include "protocol.hpp"
#include <array>
#include <unistd.h>

class personInRoom;

class MotorController {
public:

MotorController();

void moveAxisToSomeAngle1(int angleToReach);
void moveAxisToSomeAngle2(int angleToReach);
void moveAxisToSpecificAngle1(int angleToReach);
void moveAxisToSpecificAngle2(int angleToReach);
void moveAxisCw1(int axisIndex, int microseconds);
void moveAxisCw2(int axisIndex, int microseconds);
void moveAxisCcw1(int axisIndex, int microseconds);
void moveAxisCcw2(int axisIndex, int microseconds);
void moveAxisToSomeAngleI(int angleToReach, int axes);

void startAutohoming();
void startRotatyEncoders();
void readAngle();
void readStatusAngle();
void readVal(uint16_t* buf);
void writeVal(uint16_t valToWrite);
void writeVal(uint16_t address, uint16_t valToWrite);
void move_motor(int led, unsigned int time, int numberOfSteps);
void move_motor2(int led, unsigned int time, int numberOfSteps);
bool checkIfRobotIsAtHome();
bool isValidAngle(int angleToReach, int axis);

	// c m 70 999
	// if currentAngle==receivedAngle
	// sends finished command if receives the same command
	// but if some other will sends and current angle will be changed = 1 sender wont receive finishing command
	// bool checkIfMovementIsFinished() {

	// }
	void reachCcwLimit1();
	void reachCcwLimit2();

	bool startMovement() { return true; }
	//bool isThereAxisMovement[NUMBER_OF_ANGLES];
	//int axisMovementID[NUMBER_OF_ANGLES];

	std::array<int, NUMBER_OF_ANGLES> receivedIds;
	std::array<int, NUMBER_OF_ANGLES> receivedAngle;
	// std::array<int, NUMBER_OF_ANGLES> currentAngles;
	std::array<int, NUMBER_OF_ANGLES> savedReceivedAngle;
	std::array<char, MAX_IP_PACK_SIZE> textMsg;
    bool RUNNING = true;
int file_i2c;

// 1 300ccw;120;300cw //14
// 2 167ccw;77;347 //30
enum {NUMBER_OF_AXES = 2};
 bool isTheAutohomingStarted;
/////////////////////////////////axes final positions
int currentAngle [6];
bool isNegativeCurrentAngle [2];

 int currentAngleToRequest = 0;

bool direction1 = 0;
bool direction2 = 0;
bool direction3 = 0;

int stepPins[6];
int dirPins[6];
int microseconds [6];
int inc[6];
int steps[6];
bool isRobotAtHome;
bool isAxesAtHome[2];
int autohomingMicroseconds[6];
int autohomingSteps[6];
bool forwardValue[6];
bool homeWasReached[6];
bool isCcwIncreasesValueOfMagnetEncoder [6];
int currentMovementID [2];
int homePositions[6];
bool isThereMovementToSpecificAngle[6];
bool shouldInverseSignOfReceivedAngleFromClient[6];

personInRoom* personControllingAxis[2];

class AxisBorders
{
public:
  int cw;
  int ccw;
  int home;
};
AxisBorders axisBorders[6];

};

class AngleFilterMakeHomeToBeZero
{
public:

// home is 120. moving ccw 0 axis reached 80. 
// 120 - 80 = 40 = current angle if home is zero.

// home is 77, after ccw it is 97. 20 should be displayed= ccw-home= 97-77 = 20.
// after cw ca is 57, ca-home = -20 = correct. we should extract a home angle.

  int returnAngleWhereHomeAngleIsZero(int currentHomeAngle, int currentAngle, bool isCcwIncreasesValueOfMagnetEncoder) {
	if (isCcwIncreasesValueOfMagnetEncoder) 
	  return currentAngle - currentHomeAngle;
	else
	  return currentHomeAngle - currentAngle;
  }
};

// if home is zero
// 0
// cw = -130
// ccw = 150
// home = 0

//1
// cw = -80
// ccw = 80
// h = 0

