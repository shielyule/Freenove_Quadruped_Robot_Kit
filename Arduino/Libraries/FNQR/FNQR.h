/*
 * File       Class for control Freenove Quadruped Robot
 * Brief      Users can directly use this class to control Freenove Quadruped Robot.
 *            This class is very easy to use, users can use its member functions directly without any concerns.
 *            Conversions between different actions will be performed automatically.
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/11/13
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#pragma once

#include "FNQRRemote.h"

#if defined(ARDUINO_AVR_MEGA2560)

#include "FNQRComm.h"

class FNQR
{
public:
  FNQR();

 /*
  * Brief     Start the robot
  * Param     commFunction  Whether to open communication function
  *             true        The robot will be controlled by remote, Android APP or Processing APP
  *             false       You can only use the member functions of this class to control the robot
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void Start(bool commFunction = false);

 /*
  * Brief     Update the communication function
  *           If the communication function is opened, the loop() function should only call this function.
  *           Else this function should not be called.
  * Param     None
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void Update();

 /*
  * Brief     Set Wi-Fi name and password
  *           Call this function before Start()
  *           If don't call this function, will use default name and password.
  * Param     name      Wi-Fi name
  *           password  Wi-Fi password, at least 8 characters, case sensitive
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void SetWiFi(String name, String password);

 /*
  * Brief     Set wireless communication channel
  *           Call this function before Start()
  *           If don't call this function, will use default value.
  *           The remote control should set the same channel to control this robot.
  * Param     bytex     bytes to define the channel
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void SetRC(byte byte0, byte byte1, byte byte2, byte byte3, byte byte4);

 /*
  * Brief     Activate the robot
  * Param     None
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void ActiveMode();

 /*
  * Brief     Deactivate the robot, this mode is more power saving
  * Param     None
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void SleepMode();

 /*
  * Brief     Switch between active and sleep mode
  * Param     None
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void SwitchMode();

 /*
  * Brief     Crawl forward
  * Param     None
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void CrawlForward();

 /*
  * Brief     Crawl backward
  * Param     None
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void CrawlBackward();

 /*
  * Brief     Turn left
  * Param     None
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void TurnLeft();

 /*
  * Brief     Turn right
  * Param     None
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void TurnRight();

 /*
  * Brief     Move body
  * Param     x, y, z   The direction and distance want body to move
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void MoveBody(float x, float y, float z);

 /*
  * Brief     Rotate body
  * Param     x, y, z   The direction and degree want body to rotate
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void RotateBody(float x, float y, float z);

 /*
  * Brief     Twist body
  * Param     xMove, yMove, zMove           The direction and distance want body to move
  *           xRotate, yRotate, zRotate     The direction and degree want body to rotate
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void TwistBody(float xMove, float yMove, float zMove, float xRotate, float yRotate, float zRotate);

private:
  Communication communication;
};

#endif
