/*
* robot.h
* Ottomator100
*
* Created by Louis CHEN on 03/08/15
* Copyright (c) 2015 ITECH INSTRUMENTS. All rights reserved.
*/


#ifndef ROBOT_H
#define ROBOT_H

#include "busmanager.h"
#include <string>
#include "ottoutils.h"

#define POSITION_ZERO        0
// Sources holder 1..20
#define POSITION_DETECTOR   991
#define POSITION_REST       992

// Logging as global function
void writeLog(std::string text);

class Robot
{
public:
  Robot();
  ~Robot();

  // Motion methods
  int setPosition(int actuator, int position, int timeOut = 40000); //  Default timeOut = 40s
  int setCastelPositionTo(bool open, int timeOut = 40000); // Sortcut function for castel motion
  bool jog(int actuator, int direction, int duration = 500); // Default duration = 0.5s
  bool homing(int actuator);

  void alarmReset(int actuator); // acknowledgeError

  // Motion information
  int completionFor(int actuator, int position, int actionBitIndex, int timeOut);
  void updateStatusOfCurrent(int actuator, int verbose = 0); // Read and parse data
  void setRobotPosition(int actuator, int value);
  int getRobotPosition(int actuator);

  // Hardware management
  bool activateMODBUS(int actuator);
  bool servo(bool onOff, int actuator);
  bool getDeviceDataStatusRegister1Bit(int index);

  // Components
  BusManager m_busManager;

private:
  // Position states
  int positionArray[5];

  // Current actuator status
  int deviceDataStatusRegister1[16]; // (DSS1) 9005H

};

#endif // ROBOT_H
