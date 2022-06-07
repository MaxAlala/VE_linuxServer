#include <wiringPi.h>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <unistd.h>       //Needed for I2C port
#include <fcntl.h>        //Needed for I2C port
#include <sys/ioctl.h>      //Needed for I2C port
#include <linux/i2c-dev.h>    //Needed for I2C port
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include <fstream>
#include "tcpServer.h"
// global flag used to exit from the main loop
bool RUNNING = true;
int file_i2c;

// 1 300ccw;120;300cw //14
// 2 167ccw;77;347 //30
int loops[6];
const static int numberOfAxes = 1;
/////////////////////////////////axes final positions
int currentAngle [6];
bool isNegativeCurrentAngle [numberOfAxes+1];

static int currentAngleToRequest = 0;

bool direction1 = 0;
bool direction2 = 0;
bool direction3 = 0;

int stepPins[6];

int dirPins[6];

int microseconds [6];
int inc[6];
int steps[6];

int autohomingMicroseconds[6];
int autohomingSteps[6];
bool forwardValue[6];
bool homeWasReached[6];
bool isMagnetValueUpForward [6];
int homePositions[6];
bool isThereMovementToSpecAngle[6];
class AxisBorders
{
public:
  int left;
  int right;
  int home;
};
AxisBorders axisBorders[6];
void readVal(uint16_t* buf)
{
  if (read(file_i2c, buf, 1) != 1)   //read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
  {
    //ERROR HANDLING: i2c transaction failed
    printf("Failed to read from the i2c bus.\n");
  }
  else
  {
//    printf("read val: %ld\n", buf[0]);
  }
}

void writeVal(uint16_t valToWrite)
{
//  uint16_t buffer[1];
//  uint16_t addres[1];

  if (write(file_i2c, &valToWrite, 1) != 1)    //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
  {
    /* ERROR HANDLING: i2c transaction failed */
    printf("Failed to write to the i2c bus.\n");
  }
}

void writeVal(uint16_t address, uint16_t valToWrite)
{
  writeVal(address);
  writeVal(valToWrite);
}

void readAngle()
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
  length = 1;     //<<< Number of bytes to read

  readVal(&lowval);

  {
//    printf("Data read1: %ld\n", lowval);
  }

  //  for (uint8_t i = 0; i < length; i++) {
  //    printf("read val = %d \n", buffer[i]);
  //  }

  /////////////////////////////////////read high val

  //read status
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

    int curAngle = 1;
    int newVal = (lowval | highval) * 0.087890625;
    if(currentAngle[curAngle] >= 0 && currentAngle[curAngle] <= 10 && newVal > 300)
    {
      isNegativeCurrentAngle[curAngle] = true;
    }
    else if(isNegativeCurrentAngle[curAngle] && currentAngle[curAngle] < 0 && newVal >= 0 && newVal <= 60)
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

//      currentAngle[1] = newVal - 360;
//    else
//      currentAngle[1] = newVal;

//    currentAngle[5] = (lowval | highval) * 0.087890625;
  }
  else if (currentAngleToRequest == 1)
  {
    int curAngle = 0;
    int newVal = (lowval | highval) * 0.087890625;
    if(currentAngle[curAngle] >= 0 && currentAngle[curAngle] <= 10 && newVal > 300)
    {
      isNegativeCurrentAngle[curAngle] = true;
    }
    else if(isNegativeCurrentAngle[curAngle] && currentAngle[curAngle] < 0 && newVal >= 0 && newVal <= 60)
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

void readStatusAngle()
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
  length = 1;     //<<< Number of bytes to read
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


// Blink an LED
void blink_led(int led, int time)
{
  digitalWrite(led, HIGH);
  delay(time);
  digitalWrite(led, LOW);
  delay(time);
}

// Blink an LED
void move_motor(int led, unsigned int time, int numberOfSteps)
{

  for(int i = 0; i < numberOfSteps; i++)
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
void move_motor2(int led, unsigned int time, int numberOfSteps)
{

  for(int i = 0; i < numberOfSteps; i++)
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
void move_motor3(int led, unsigned int time, int numberOfSteps)
{

  for(int i = 0; i < numberOfSteps; i++)
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
void move_motor4(int led, unsigned int time, int numberOfSteps)
{

  for(int i = 0; i < numberOfSteps; i++)
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
void move_motor5(int led, unsigned int time, int numberOfSteps)
{
  for(int i = 0; i < numberOfSteps; i++)
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
void move_motor6(int led, unsigned int time, int numberOfSteps)
{
  for(int i = 0; i < numberOfSteps; i++)
  {
    digitalWrite(led, HIGH);
//    usleep(time);
    std::this_thread::sleep_for(std::chrono::nanoseconds(time));
    digitalWrite(led, LOW);
//    usleep(time);
    std::this_thread::sleep_for(std::chrono::nanoseconds(time));
  }
}

void moveMotorByIndex(int motorToMove, bool shouldGoForwardFirst = false)
{
  if (shouldGoForwardFirst)
  {
    digitalWrite(dirPins[motorToMove],forwardValue[motorToMove]);
    move_motor(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);

//    for(int i = 0; i < steps[motorToMove]; i++)
//    {
//      digitalWrite(stepPins[motorToMove], HIGH);
//      std::this_thread::sleep_for(std::chrono::nanoseconds(microseconds[motorToMove]));
//      digitalWrite(stepPins[motorToMove], LOW);
//      std::this_thread::sleep_for(std::chrono::nanoseconds(microseconds[motorToMove]));
//    }

    digitalWrite(dirPins[motorToMove], !forwardValue[motorToMove]);
    move_motor(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
//    for(int i = 0; i < steps[motorToMove]; i++)
//    {
//      digitalWrite(stepPins[motorToMove], HIGH);
//      std::this_thread::sleep_for(std::chrono::nanoseconds(microseconds[motorToMove]));
//      digitalWrite(stepPins[motorToMove], LOW);
//      std::this_thread::sleep_for(std::chrono::nanoseconds(microseconds[motorToMove]));
//    }
  }
  else
  {
    digitalWrite(dirPins[motorToMove],!forwardValue[motorToMove]);
    move_motor(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], forwardValue[motorToMove]);
    move_motor(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
}

void moveMotorByIndex2(int motorToMove, bool shouldGoForwardFirst = false)
{
  if (shouldGoForwardFirst)
  {
    digitalWrite(dirPins[motorToMove],forwardValue[motorToMove]);
    move_motor2(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], !forwardValue[motorToMove]);
    move_motor2(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
  else
  {
    digitalWrite(dirPins[motorToMove],!forwardValue[motorToMove]);
    move_motor2(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], forwardValue[motorToMove]);
    move_motor2(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
}

void moveMotorByIndex3(int motorToMove, bool shouldGoForwardFirst = false)
{
  if (shouldGoForwardFirst)
  {
    digitalWrite(dirPins[motorToMove],forwardValue[motorToMove]);
    move_motor3(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], !forwardValue[motorToMove]);
    move_motor3(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
  else
  {
    digitalWrite(dirPins[motorToMove],!forwardValue[motorToMove]);
    move_motor3(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], forwardValue[motorToMove]);
    move_motor3(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
}

void moveMotorByIndex4(int motorToMove, bool shouldGoForwardFirst = false)
{
  if (shouldGoForwardFirst)
  {
    digitalWrite(dirPins[motorToMove],forwardValue[motorToMove]);
    move_motor4(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], !forwardValue[motorToMove]);
    move_motor4(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
  else
  {
    digitalWrite(dirPins[motorToMove],!forwardValue[motorToMove]);
    move_motor4(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], forwardValue[motorToMove]);
    move_motor4(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
}
void moveMotorByIndex5(int motorToMove, bool shouldGoForwardFirst = false)
{
  if (shouldGoForwardFirst)
  {
    digitalWrite(dirPins[motorToMove],forwardValue[motorToMove]);
    move_motor5(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], !forwardValue[motorToMove]);
    move_motor5(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
  else
  {
    digitalWrite(dirPins[motorToMove],!forwardValue[motorToMove]);
    move_motor5(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], forwardValue[motorToMove]);
    move_motor5(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
}
void moveMotorByIndex6(int motorToMove, bool shouldGoForwardFirst = false)
{
  if (shouldGoForwardFirst)
  {
    digitalWrite(dirPins[motorToMove],forwardValue[motorToMove]);
    move_motor6(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], !forwardValue[motorToMove]);
    move_motor6(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
  else
  {
    digitalWrite(dirPins[motorToMove],!forwardValue[motorToMove]);
    move_motor6(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
    digitalWrite(dirPins[motorToMove], forwardValue[motorToMove]);
    move_motor6(stepPins[motorToMove], microseconds[motorToMove], steps[motorToMove]);
  }
}

void moveAxisForward1(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], forwardValue[axisIndex]);
  move_motor(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back

}

void moveAxisBack1(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], !forwardValue[axisIndex]);
  move_motor(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back
}

void moveAxisForward2(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], forwardValue[axisIndex]);
  move_motor2(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back

}

void moveAxisBack2(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], !forwardValue[axisIndex]);
  move_motor2(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back
}

void moveAxisForward3(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], forwardValue[axisIndex]);
  move_motor3(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back

}

void moveAxisBack3(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], !forwardValue[axisIndex]);
  move_motor3(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back
}

void moveAxisForward4(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], forwardValue[axisIndex]);
  move_motor4(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back

}

void moveAxisBack4(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], !forwardValue[axisIndex]);
  move_motor4(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back
}
void moveAxisForward5(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], forwardValue[axisIndex]);
  move_motor5(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back

}

void moveAxisBack5(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], !forwardValue[axisIndex]);
  move_motor5(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back
}
void moveAxisForward6(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], forwardValue[axisIndex]);
  move_motor6(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back

}

void moveAxisBack6(int axisIndex)
{
  digitalWrite(dirPins[axisIndex], !forwardValue[axisIndex]);
  move_motor6(stepPins[axisIndex], autohomingMicroseconds[axisIndex], autohomingSteps[axisIndex]); //high = back
}

// 1 300b;330;330 //30
// 2 38f;52;66  //14
// 3 292f;316;340 //24
// 4 95;95;125b
// 5 354;8;22f
// 6 344;8;32f

//0 =52 = 278b;330 = 9000 steps
//1 4000=14=38f;52;58=6=3000
//2 6000=298f=316=338=6000=can more
//3 9000=56s=151b=95
//4 4000=14=22f;8;-2=3000=10
//5 5500=24f=8=-20=28=12000
void moveAxisToSpecificAngle1(int angleToReach)
{
  int currentAxis = 0;
  if (angleToReach < axisBorders[currentAxis].left || angleToReach > axisBorders[currentAxis].right)
    return;

  isThereMovementToSpecAngle[currentAxis] = true;
  while((currentAngle[currentAxis] != angleToReach) && isThereMovementToSpecAngle[currentAxis])
  {
    if(currentAngle[currentAxis] < angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisForward1(currentAxis);
      }
      else
      {
        moveAxisBack1(currentAxis);
      }
    }
    else if(currentAngle[currentAxis] > angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisBack1(currentAxis);
      }
      else
      {
        moveAxisForward1(currentAxis);
      }
    }
  }
  isThereMovementToSpecAngle[currentAxis] = false;
}

void moveAxisToSpecificAngle2(int angleToReach)
{
  int currentAxis = 1;
  if (angleToReach < axisBorders[currentAxis].left || angleToReach > axisBorders[currentAxis].right)
    return;

  while(currentAngle[currentAxis] != angleToReach)
  {
    if(currentAngle[currentAxis] < angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisForward2(currentAxis);
      }
      else
      {
        moveAxisBack2(currentAxis);
      }
    }
    else if(currentAngle[currentAxis] > angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisBack2(currentAxis);
      }
      else
      {
        moveAxisForward2(currentAxis);
      }
    }
  }
}

void moveAxisToSpecificAngle3(int angleToReach)
{
  int currentAxis = 2;
  if (angleToReach < axisBorders[currentAxis].left || angleToReach > axisBorders[currentAxis].right)
    return;

  while(currentAngle[currentAxis] != angleToReach)
  {
    if(currentAngle[currentAxis] < angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisForward3(currentAxis);
      }
      else
      {
        moveAxisBack3(currentAxis);
      }
    }
    else if(currentAngle[currentAxis] > angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisBack3(currentAxis);
      }
      else
      {
        moveAxisForward3(currentAxis);
      }
    }
  }
}

void moveAxisToSpecificAngle4(int angleToReach)
{
  int currentAxis = 3;
  if (angleToReach < axisBorders[currentAxis].left || angleToReach > axisBorders[currentAxis].right)
    return;

  while(currentAngle[currentAxis] != angleToReach)
  {
    if(currentAngle[currentAxis] < angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisForward4(currentAxis);
      }
      else
      {
        moveAxisBack4(currentAxis);
      }
    }
    else if(currentAngle[currentAxis] > angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisBack4(currentAxis);
      }
      else
      {
        moveAxisForward4(currentAxis);
      }
    }
  }
}

void moveAxisToSpecificAngle5(int angleToReach)
{
  int currentAxis = 4;
  if (angleToReach < axisBorders[currentAxis].left || angleToReach > axisBorders[currentAxis].right)
    return;

  while(currentAngle[currentAxis] != angleToReach)
  {
    if(currentAngle[currentAxis] < angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisForward5(currentAxis);
      }
      else
      {
        moveAxisBack5(currentAxis);
      }
    }
    else if(currentAngle[currentAxis] > angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisBack5(currentAxis);
      }
      else
      {
        moveAxisForward5(currentAxis);
      }
    }
  }
}

void moveAxisToSpecificAngle6(int angleToReach)
{
  int currentAxis = 5;

  if (angleToReach < axisBorders[currentAxis].left || angleToReach > axisBorders[currentAxis].right)
    return;

  while(currentAngle[currentAxis] != angleToReach)
  {
    if(currentAngle[currentAxis] < angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisForward6(currentAxis);
      }
      else
      {
        moveAxisBack6(currentAxis);
      }
    }
    else if(currentAngle[currentAxis] > angleToReach)
    {
      if(isMagnetValueUpForward[currentAxis])
      {
        moveAxisBack6(currentAxis);
      }
      else
      {
        moveAxisForward6(currentAxis);
      }
    }
  }
}

void moveAxisToSomeAngle1(int angleToReach)
{
// stop current movement if it exists
  if (isThereMovementToSpecAngle[0] == true)
    isThereMovementToSpecAngle[0] = false;
// sleep 100ms
  usleep(100000);
  auto moveToSpecificAngle1 = [angleToReach]()
  {
    moveAxisToSpecificAngle1(angleToReach);
  };
  std::thread threadMoveToSpecificAngle1(moveToSpecificAngle1);
  threadMoveToSpecificAngle1.detach();
}

int main()
{

// Initialize wiringPi and allow the use of BCM pin numbering
  wiringPiSetupGpio();

  std::cout << "Controlling the GPIO pins with wiringPi\n";

  stepPins[0] = 17;
  stepPins[1] = 23;
  stepPins[2] = 22;
  stepPins[3] = 19;
  stepPins[4] = 6;
  stepPins[5] = 20;

  dirPins[0] = 18;
  dirPins[1] = 24;
  dirPins[2] = 27;
  dirPins[3] = 26;
  dirPins[4] = 13;
  dirPins[5] = 21;
//  pinMode(dirPins[5], OUTPUT);
//  pinMode(stepPins[5], OUTPUT);
//  pinMode(dirPins[4], OUTPUT);
//  pinMode(stepPins[4], OUTPUT);
//  pinMode(dirPins[3], OUTPUT);
//  pinMode(stepPins[3], OUTPUT);
//  pinMode(dirPins[2], OUTPUT);
//  pinMode(stepPins[2], OUTPUT);
  pinMode(dirPins[1], OUTPUT);
  pinMode(stepPins[1], OUTPUT);
  pinMode(dirPins[0], OUTPUT);
  pinMode(stepPins[0], OUTPUT);

//  digitalWrite(dirPins[5], LOW);
//  digitalWrite(dirPins[4], LOW);
//  digitalWrite(dirPins[3], LOW);
//  digitalWrite(dirPins[2], LOW);
  digitalWrite(dirPins[1], LOW);
  digitalWrite(dirPins[0], LOW);

  int speedIncreaser = 1;
  int speedDecreaser = 6;

  isNegativeCurrentAngle[0] = false;
  isNegativeCurrentAngle[1] = false;

  microseconds[0] = 50000/speedIncreaser*speedDecreaser;
  microseconds[1] = 50000/speedIncreaser*speedDecreaser;
  microseconds[2] = 50000/speedIncreaser*speedDecreaser;
  microseconds[3] = 50000/speedIncreaser*speedDecreaser;
  microseconds[4] = 50000/speedIncreaser*speedDecreaser;
  microseconds[5] = 50000/speedIncreaser*speedDecreaser;


  inc[0] = 1;
  inc[1] = 1;
  inc[2] = 1;
  inc[3] = 1;
  inc[4] = 1;
  inc[5] = 1;

  steps[0] = 3000 * inc[0];
  steps[1] = 3000 * inc[1];
  steps[2] = 3000 * inc[2];
  steps[3] = 3000 * inc[3];
  steps[4] = 3000 * inc[4];
  steps[5] = 3000 * inc[5];

  speedDecreaser = 1;
  autohomingMicroseconds[0] = 50000/speedIncreaser*speedDecreaser;
  autohomingMicroseconds[1] = 50000/speedIncreaser*speedDecreaser;
  autohomingMicroseconds[2] = 50000/speedIncreaser*speedDecreaser;
  autohomingMicroseconds[3] = 50000/speedIncreaser*speedDecreaser;
  autohomingMicroseconds[4] = 50000/speedIncreaser*speedDecreaser;
  autohomingMicroseconds[5] = 50000/speedIncreaser*speedDecreaser;

  int autohomingVal = 100;
  autohomingSteps[0] = autohomingVal;
  autohomingSteps[1] = autohomingVal;
  autohomingSteps[2] = autohomingVal;
  autohomingSteps[3] = autohomingVal;
  autohomingSteps[4] = autohomingVal;
  autohomingSteps[5] = autohomingVal;

  homeWasReached[0] = false;
  homeWasReached[1] = false;
  homeWasReached[2] = false;
  homeWasReached[3] = false;
  homeWasReached[4] = false;
  homeWasReached[5] = false;

  forwardValue[0] = false; // ccw
  forwardValue[1] = true; //ccw
  forwardValue[2] = true;
  forwardValue[3] = false;
  forwardValue[4] = false;
  forwardValue[5] = true;
  // 1 300b;330;330 //30
// 2 38f;52;66  //14
// 3 292f;316;340 //24
// 4 95;95;125b
// 5 354;8;22f
// 6 344;8;32f

//0 =52 = 278b-330 = 9000 steps
//1 4000=14=38f;52;58=6=3000
//2 6000=298f=316=338=6000=can more
//3 9000=56s=151b=95
//4 4000=14=22f;8;-2=3000=10
//5 5500=24f=8=-20=28=12000

// 1 300ccw;120;300cw //14
// 2 347ccw;77;167//30

  axisBorders[1].left = -13;
  axisBorders[1].right = 167;
  axisBorders[1].home = 77;

  axisBorders[0].left = -60;
  axisBorders[0].right = 300;
  axisBorders[0].home = 120;

//  axisBorders[2].left = 182;
//  axisBorders[2].right = 224;
//  axisBorders[2].home = 200;
//
//  axisBorders[3].left = 95;
//  axisBorders[3].right = 151;
//  axisBorders[3].home = 95;
//
//  axisBorders[4].left = -2;
//  axisBorders[4].right = 22;
//  axisBorders[4].home = 8;
//
//  axisBorders[5].left = -20;
//  axisBorders[5].right = 8;
//  axisBorders[5].home = 24;
//
//  int stepsForward[3];


  homePositions[0]=120;
  homePositions[1]=77;
//  homePositions[2]=200;
//  homePositions[3]=95;
//  homePositions[4]=8;
//  homePositions[5]=8;

//  stepsForward[0] = 30;
//  stepsForward[1] = 14;
//  stepsForward[2] = 24;

  isMagnetValueUpForward[0] = true;  //+
  isMagnetValueUpForward[1] = false; //+
//  isMagnetValueUpForward[2] = false; //+
//  isMagnetValueUpForward[3] = false; //
//  isMagnetValueUpForward[4] = true;  //
//  isMagnetValueUpForward[5] = true;  //

  isThereMovementToSpecAngle[0] = false;
  isThereMovementToSpecAngle[1] = false;
//  isThereMovementToSpecAngle[2] = false;
//  isThereMovementToSpecAngle[3] = false;
//  isThereMovementToSpecAngle[4] = false;
//  isThereMovementToSpecAngle[5] = false;

  //----- OPEN THE I2C BUS -----
  char *filename = (char*)"/dev/i2c-1";
  if ((file_i2c = open(filename, O_RDWR)) < 0)
  {
    //ERROR HANDLING: you can check errno to see what went wrong
    printf("Failed to open the i2c bus");
    return -1;
  }

  typedef struct
  {
    uint16_t regAddr;
    uint8_t L2_value;
    uint8_t cmd;
  } __attribute__((__packed__)) Modify_struct;


  while(true)
  {

    auto readAngle1 = []()
    {
      while (true)
      {
        int MUX_address = 0x70;          //<<<<<The I2C address of the slave

        if (ioctl(file_i2c, I2C_SLAVE, MUX_address) < 0)
        {
          printf("Failed to acquire bus access and/or talk to slave.\n");
          return -1;
        }

        writeVal(MUX_address, 1 << currentAngleToRequest);

        const int as5600_address = 0x36;
        if(ioctl(file_i2c, I2C_SLAVE, as5600_address) < 0)
        {
          printf("Fail to reach laser \n");
          exit(EXIT_FAILURE);
        }

        readStatusAngle();
        if(currentAngleToRequest == numberOfAxes)
          currentAngleToRequest = 0;
        else ++currentAngleToRequest;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    };

    auto displayAngles = []()
    {
      unsigned int microsecond1 = 1500000;
      while (true)
      {
        printf("angle1=%ld, angle2=%ld\n", currentAngle[0], currentAngle[1]);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    };

    auto moveMotors = [dirPins,microseconds,stepPins,steps]()
    {
//      unsigned int-microsecond1 = 1500000;
      while (true)
      {

////////////////////////////////////////////// move first axis between its final positions
//1 axis
//while(direction1 == 0) {
//    digitalWrite(dir1, HIGH);
//    move_motor(step1, microsecond1, steps1); //high = back
//    std::cout << "dir1=0 \n";
//    if(finalAngle1 <= 300 && finalAngle1 > 260) {
//        direction1 = 1;
//    }
//}
//
//while(direction1 == 1) {
//    digitalWrite(dir1, LOW);
//    move_motor(step1, microsecond1, steps1); //high = back
//    std::cout << "dir1=1 \n";
//    if(finalAngle1 >= 330 && finalAngle1 < 360) {
//        direction1 = 0;
//    }
//}

////2 axis
//while (direction2 == 0) {
//    digitalWrite(dir2, HIGH);
//    move_motor(step2, microsecond2, steps2); //high = back
//    std::cout << "dir2=0 \n";
////    file_out << 0 << "\n";
//    if (finalAngle2 <= 30 && finalAngle2 > 20) {
//      direction2 = 1;
//    }
//}
//
//while (direction2 == 1) {
//    digitalWrite(dir2, LOW);
//    move_motor(step2, microsecond2, steps2); //high = back
//    std::cout << "dir2=1\n";
//
////    file_out << 1 << "\n";
//    if (finalAngle2 >= 50 && finalAngle2 < 90) {
//        direction2 = 0;
//    }
//}
//

////3 axis
//while (direction3 == 0) {
//    digitalWrite(dir3, HIGH);
//    move_motor(step3, microsecond3, steps3); //high = back
//    std::cout << "dir3=0 \n";
//
//    if (finalAngle3 <= 305 && finalAngle3 > 270) {
//      direction3 = 1;
//    }
//}
//
//while (direction3 == 1) {
//    digitalWrite(dir3, LOW);
//    move_motor(step3, microsecond3, steps3); //high = back
//    std::cout << "dir3=1\n";
//
//    if (finalAngle3 >= 325 && finalAngle3 < 360) {
//        direction3 = 0;
//    }
//}



//// 1 300b;330;330 //30
// 2 38f;52;66  //14
// 3 292f;316;340 //24
// 4 95;95;125b
// 5 354;8;22f
// 6 344;8;32f

        moveMotorByIndex(0, false);

//        digitalWrite(dirPins[3],HIGH); // L = forward
//        move_motor(stepPins[3], microseconds[3], steps[3]);
//        digitalWrite(dirPins[3], LOW);
//        move_motor(stepPins[3], microseconds[3], steps[3]);
        usleep(2000000);
      }
    };

    auto moveMotors2 = [dirPins,microseconds,stepPins,steps]()
    {
//      unsigned int-microsecond1 = 1500000;
      while (true)
      {

//// 1 300b;330;330 //30
// 2 38f;52;66  //14
// 3 292f;316;340 //24
// 4 95;95;125b
// 5 354;8;22f
// 6 344;8;32f

        moveMotorByIndex2(1, true);

//        digitalWrite(dirPins[3],HIGH); // L = forward
//        move_motor(stepPins[3], microseconds[3], steps[3]);
//        digitalWrite(dirPins[3], LOW);
//        move_motor(stepPins[3], microseconds[3], steps[3]);
        usleep(2000000);
      }
    };

    auto moveMotors3 = [dirPins,microseconds,stepPins,steps]()
    {
      while (true)
      {
        moveMotorByIndex3(2, true);
        usleep(2000000);
      }
    };

    auto moveMotors4 = [dirPins,microseconds,stepPins,steps]()
    {
      while (true)
      {
        moveMotorByIndex4(3, false);
        usleep(2000000);
      }
    };

    auto moveMotors5 = [dirPins,microseconds,stepPins,steps]()
    {
      while (true)
      {
        moveMotorByIndex5(4, true);
        usleep(2000000);
      }
    };

    auto moveMotors6 = [dirPins,microseconds,stepPins,steps]()
    {
      while (true)
      {
        moveMotorByIndex6(5, true);
        usleep(2000000);
      }
    };



//    bool isMagnetBackValueLooped[6];
//    // back looped val goes from 0 to > 330 = destroys this if = if curVal 8 > homePosition {go back} = problem
//    // if back looped value = if val > 300 = magnetic value goes up = takes oppo
//
//    isMagnetBackValueLooped[0] = false; //
//    isMagnetBackValueLooped[1] = false; //
//    isMagnetBackValueLooped[2] = false; //
//    isMagnetBackValueLooped[3] = false; //
//    isMagnetBackValueLooped[4] = true; //
//    isMagnetBackValueLooped[5] = true; //
// 1 300b;330;330 //30
// 2 38f;52;66  //14
// 3 292f;316;340 //24
// 4 95;95;125b
// 5 354;8;22f
// 6 344;8;32f



    auto autohoming0 = []()
    {
      while (true)
      {
        int currentAxis = 0;
        while(currentAngle[currentAxis] != homePositions[currentAxis])
        {
          if(currentAngle[currentAxis] < homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisForward1(currentAxis);
            }
            else
            {
              moveAxisBack1(currentAxis);
            }
          }
          else if(currentAngle[currentAxis] > homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisBack1(currentAxis);

            }
            else
            {
              moveAxisForward1(currentAxis);
            }
          }
        }
//        usleep(2000000);
      }
    };

    auto autohoming1 = []()
    {
      while (true)
      {
        int currentAxis = 1;
        while(currentAngle[currentAxis] != homePositions[currentAxis])
        {
          if(currentAngle[currentAxis] < homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisForward2(currentAxis);
            }
            else
            {
              moveAxisBack2(currentAxis);
            }
          }
          else if(currentAngle[currentAxis] > homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisBack2(currentAxis);

            }
            else
            {
              moveAxisForward2(currentAxis);
            }
          }
        }
      }
    };

    auto autohoming2 = [homePositions,isMagnetValueUpForward]()
    {
      while (true)
      {
        int currentAxis = 2;
        while(currentAngle[currentAxis] != homePositions[currentAxis])
        {
          if(currentAngle[currentAxis] < homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisForward2(currentAxis);
            }
            else
            {
              moveAxisBack2(currentAxis);
            }
          }
          else if(currentAngle[currentAxis] > homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisBack2(currentAxis);

            }
            else
            {
              moveAxisForward2(currentAxis);
            }
          }
        }
      }
    };


    auto autohoming3 = [homePositions,isMagnetValueUpForward]()
    {
      while (true)
      {
        int currentAxis = 3;
        while(currentAngle[currentAxis] != homePositions[currentAxis])
        {
          if(currentAngle[currentAxis] < homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisForward3(currentAxis);
            }
            else
            {
              moveAxisBack3(currentAxis);
            }
          }
          else if(currentAngle[currentAxis] > homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisBack3(currentAxis);

            }
            else
            {
              moveAxisForward3(currentAxis);
            }
          }
        }
      }
    };



    auto autohoming4 = [homePositions,isMagnetValueUpForward]()
    {
      while (true)
      {
        int currentAxis = 4;
        while(currentAngle[currentAxis] != homePositions[currentAxis])
        {
          if(currentAngle[currentAxis] < homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisForward4(currentAxis);
            }
            else
            {
              moveAxisBack4(currentAxis);
            }
          }
          else if(currentAngle[currentAxis] > homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisBack4(currentAxis);

            }
            else
            {
              moveAxisForward4(currentAxis);
            }
          }
        }
      }
    };


    auto autohoming5 = [homePositions,isMagnetValueUpForward]()
    {
      while (true)
      {
        int currentAxis = 5;
        while(currentAngle[currentAxis] != homePositions[currentAxis])
        {
          if(currentAngle[currentAxis] < homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisForward5(currentAxis);
            }
            else
            {
              moveAxisBack5(currentAxis);
            }
          }
          else if(currentAngle[currentAxis] > homePositions[currentAxis])
          {
            if(isMagnetValueUpForward[currentAxis])
            {
              moveAxisBack5(currentAxis);

            }
            else
            {
              moveAxisForward5(currentAxis);
            }
          }
        }
      }
    };

    // 1 300b;330;330 //30
// 2 38f;52;66  //14
// 3 292f;316;340 //24
// 4 95;95;125b
// 5 354;8;22f
// 6 344;8;32f

//0 =52 = 278b-330 = 9000 steps
//1 4000=14=38f;52;58=6=3000
//2 6000=298f=316=338=6000=can more
//3 9000=56s=151b=95
//4 4000=14=22f;8;-2=3000=10
//5 5500=24f=8=-20=28=12000


    auto moveToSpecificAngle2 = []()
    {
      moveAxisToSpecificAngle2(330);
    };

    auto moveToSpecificAngle3 = []()
    {
      moveAxisToSpecificAngle3(330);
    };

    auto moveToSpecificAngle4 = []()
    {
      moveAxisToSpecificAngle4(330);
    };
    auto moveToSpecificAngle5 = []()
    {
      moveAxisToSpecificAngle5(330);
    };

    auto moveAxesToSpecificAngles = []()
    {
      moveAxisToSomeAngle1(60);
      usleep(2000000);
      moveAxisToSomeAngle1(180);
    };




// motor has high and low
// there is physical forward value = leg goes forward = forwardValue has drivers high or low
// there is magnet forward value = leg goes forward but magnetic value decreases = thats why i need isVUpForward. If leg goes  forward + val goes up then true

    // 1 300b;330;330 //30
// 2 38f;52;66  //14
// 3 292f;316;340 //24
// 4 95;95;125b
// 5 354;8;22f
// 6 344;8;32f

//0 =52 = 278b-330 = 9000 steps
//1 4000=14=38f;52;58=6=3000
//2 6000=298f=316=338=6000=can more
//3 9000=56s=151b=95
//4 4000=14=22f;8;-2=3000=10
//5 5500=24f=8=-20=28=12000

	ServerController serverController;
	serverController.startServer();
//    std::thread threadMoveAxesToSpecificAngles(moveAxesToSpecificAngles);

//    steps[0] = 8000;
//    steps[1] = 3000;
//    steps[2] = 3000 * 2;
//    steps[3] = 9000;
//    steps[4] = 3000 * inc[4];
//    steps[5] = 12000 * inc[5];

//    std::thread threadAutohoming2(autohoming2);
//    std::thread threadAutohoming3(autohoming3);
//    std::thread threadAutohoming4(autohoming4);
//    std::thread threadAutohoming5(autohoming5);
    std::thread threadReadAngle1(readAngle1);
    std::thread threadDisplayAngles(displayAngles);
//    std::thread threadMoveMotor(moveMotors);
//    std::thread threadMoveMotor2(moveMotors2);
//    std::thread threadMoveMotor3(moveMotors3);
//    std::thread threadMoveMotor4(moveMotors4);
//    std::thread threadMoveMotor5(moveMotors5);
//    std::thread threadMoveMotor6(moveMotors6);



//home position
//1=330,2=52,3=316,4=95,5=8,6=8
//    threadMoveAxesToSpecificAngles.join();
   threadAutohoming0.join();
   threadAutohoming1.join();
//    threadAutohoming2.join();
//    threadAutohoming3.join();
//    threadAutohoming4.join();
//    threadAutohoming5.join();
    threadReadAngle1.join();
    threadDisplayAngles.join();
//    threadMoveMotor.join();
//    threadMoveMotor2.join();
//    threadMoveMotor3.join();
//    threadMoveMotor4.join();
//    threadMoveMotor5.join();
//    threadMoveMotor6.join();
//300-330,
//30-50
//305-325

// 1 300b;330;330 //30
// 2 38f;52;66  //14
// 3 292f;316;340 //24
// 4 95;95;125b
// 5 354;8;22f
// 6 344;8;32f

//while (direction3 == 0) {
//    digitalWrite(dir3, HIGH);
//    move_motor(step3, microsecond3, steps3); //high = back
//    std::cout << "dir3=0 \n";
//
//    if (finalAngle3 <= 305 && finalAngle3 > 270) {
//      direction3 = 1;
//    }
//}
//
//while (direction3 == 1) {
//    digitalWrite(dir3, LOW);
//    move_motor(step3, microsecond3, steps3); //high = back
//    std::cout << "dir3=1\n";
//
//    if (finalAngle3 >= 325 && finalAngle3 < 360) {
//        direction3 = 0;
//    }
//}




//    auto f1 = [steps1,step1,dir1,microsecond1]()
//    {
////      while (true)
//      {
//        digitalWrite(dir1, HIGH);
//        move_motor(step1, microsecond1, steps1); //down
//        digitalWrite(dir1, LOW);
//        move_motor(step1, microsecond1, steps1); //down
////        usleep(2000000);
//      }
//    };
//
////    auto f2 = []()
////    {
////      while (true)
////      {
////        digitalWrite(dir2, HIGH);
////        move_motor(step2, microsecond2, steps2); //down
////        digitalWrite(dir2, LOW);
////        move_motor(step2, microsecond2, steps2); //down
////        usleep(2000000);
////      }
////    };
//
//    auto f3 = [steps3,step3,dir3,microsecond3]()
//    {
////      while (true)
//      {
////        digitalWrite(dir3, LOW);//back
////        move_motor(step3, microsecond3, steps3);
//        digitalWrite(dir3, HIGH); // forward
//        move_motor(step3, microsecond3, steps3);
//        usleep(2000000);
//      }
//    };
//
//    auto f4 = [steps4,step4,dir4,microsecond4]()
//    {
////      while (true)
//      {
//        digitalWrite(dir4, LOW);
//        move_motor4(step4, microsecond4, steps4); //down
//
//        digitalWrite(dir4, HIGH);
//        move_motor4(step4, microsecond4, steps4); //down
////        usleep(2000000);
//      }
//    };
//
//    auto f5 = [steps5,step5,dir5,microsecond5]()
//    {
////      while (true)
//      {
//        digitalWrite(dir5, HIGH);
//        move_motor5(step5, microsecond5, steps5); //down
//        digitalWrite(dir5, LOW);
//        move_motor5(step5, microsecond5, steps5); //down
//        usleep(2000000);
//      }
//    };
//
//    auto f6 = [steps6,step6,dir6,microsecond6]()
//    {
////      while (true)
//      {
////        digitalWrite(dir6, HIGH);
////        move_motor6(step6, microsecond6, steps6); //down
//        digitalWrite(dir6, LOW);
//        move_motor6(step6, microsecond6, steps6); //down
//        usleep(2000000);
//      }
//    };
//    std::thread m1(f1);
////    std::thread m2(f2);
//    std::thread m3(f3);
//    std::thread m4(f4);
////    std::thread m5(f5);
//    std::thread m6(f6);
//
//    m1.join();
////    m2.join();
//    m3.join();
//    m4.join();
//    m5.join();
//    m6.join();
//    m4.join();
//

// 19  vs 347
//

//[347; 19]
///////////////////////////////1 axis goes back


//    digitalWrite(dir2, LOW); // low = out
//    move_motor(step2, microsecond2, steps2/divider); //low =
//    digitalWrite(dir3,LOW); // low = forward
//    move_motor(step3, microsecond3, steps3); //downs


//    digitalWrite(dir4, HIGH); // high =forward
//    move_motor(step4, microsecond4, steps4/divider); //down

//    digitalWrite(dir4, second4, steps4/divider); //down
//    digitalWrite(dir5, LOW); // low = out
//    move_motor(step5, microsecond5, steps5); //down
//    digitalWrite(dir5, HIGH); // high = in
//    move_motor(step5, microsecond5, steps5); //down

//    digitalWrite(dir6, LOW); // low = back
//    move_motor(step6, microsecond6, steps6/divider); //down
//            usleep(2000000);
//        digitalWrite(dir1, LOW);
//        move_motor(step1, microsecond1, steps1); //down
//            usleep(2000000);
//        digitalWrite(dir3, LOW);
//        move_motor(step3, microsecond3, steps3); //down
//    usleep(4000000);
//// cw
//    digitalWrite(dir1, HIGH);
//    move_motor(step1, microsecond1, steps1/divider); //down
//
//    digitalWrite(dir1, LOW);
//    move_motor(step1, microsecond1, steps1/divider); //down




////cw
//    digitalWrite(dir3, LOW);
//    move_motor(step3, microsecond3, steps3/divider); //down
////ccw
//    digitalWrite(dir3, HIGH);
//    move_motor(step3, microsecond3, steps3/divider); //down

////cw
//    digitalWrite(dir4, LOW);
//    move_motor(step4, microsecond4, steps4/divider); //down
////ccw
//    digitalWrite(dir4, HIGH);
//    move_motor(step4, microsecond4, steps4/divider); //down

////cw
//    digitalWrite(dir5, LOW);
//    move_motor(step5, microsecond5, steps5/divider); //down
////ccw
//    digitalWrite(dir5, HIGH);
//    move_motor(step5, microsecond5, steps5/divider); //down
////cw
//    digitalWrite(dir6, LOW);
//    move_motor(step6, microsecond6, steps6/divider); //down
////ccw
//    digitalWrite(dir6, HIGH);
//    move_motor(step6, microsecond6, steps6/divider); //down


// to restore 8000 high = should do 11000 low + 3000 high to get balance

//      blink_led(red, time);
//    blink_led(yellow, time);
//    blink_led(green, time);
  }

  std::cout << "Program ended ...\n";
}

