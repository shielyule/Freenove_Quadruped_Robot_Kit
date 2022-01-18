/*
 * File       Communication class for Freenove Quadruped Robot
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/11/13
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#if defined(ARDUINO_AVR_MEGA2560)

#include "FNQRComm.h"

Communication* communication = NULL;

Communication::Communication() {}

void Communication::Start(bool commFunction)
{
  this->commFunction = commFunction;

  StartStateLed();

  robotAction.Start();

  communication = this;

  FlexiTimer2::set(20, UpdateService);
  FlexiTimer2::start();

  if (commFunction)
  {
    StartPins();
    StartSerial();
    StartRF24();
    StartESP8266();

    SetRobotBootState(GetRobotBootState());
  }

  if (robotAction.robot.power.powerGroupAutoSwitch)
    delay(1000);
}

void Communication::SetWiFi(String name, String password)
{
  esp8266SSID = name;
  esp8266PWD = password;
}

void Communication::SetRC(byte byte0, byte byte1, byte byte2, byte byte3, byte byte4)
{
  rf24Address[0] = byte0;
  rf24Address[1] = byte1;
  rf24Address[2] = byte2;
  rf24Address[3] = byte3;
  rf24Address[4] = byte4;
}

void Communication::StartStateLed()
{
  pinMode(stateLedPin, OUTPUT);
  digitalWrite(stateLedPin, LOW);
}

void Communication::SetStateLed(bool state)
{
  digitalWrite(stateLedPin, state);
}

void Communication::ReverseStateLed()
{
  stateLedState = !stateLedState;
  SetStateLed(stateLedState);
}

void Communication::StartPins()
{
  for (int i = 0; i < 8; i++)
  {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

void Communication::StartSerial()
{
  Serial.begin(115200);
}

void Communication::UpdateSerial()
{
  while (Serial.available() > 0)
  {
    byte inByte = Serial.read();
    if (inByte == Orders::transStart)
      serialInDataCounter = 0;
    serialInData[serialInDataCounter++] = inByte;
    if (inByte == Orders::transEnd)
      break;
  }

  if (serialInData[0] == Orders::transStart && serialInData[serialInDataCounter - 1] == Orders::transEnd)
  {
    HandleOrder(serialInData, OrderSource::FromSerial);
    serialInDataCounter = 0;
  }
}

void Communication::StartRF24()
{
  pinMode(50, INPUT_PULLUP);
  isRF24Available = rf24.begin();
  if (!isRF24Available)
    return;
  rf24.setPALevel(RF24_PA_LOW);
  rf24.setDataRate(RF24_1MBPS);
  rf24.setRetries(0, 15);
  rf24.enableDynamicPayloads();
  rf24.openWritingPipe(rf24Address);
  rf24.openReadingPipe(1, rf24Address);
  rf24.startListening();
}

void Communication::UpdateRF24()
{
  if (!isRF24Available)
    return;

  if (!rf24.available())
    return;

  int inDataCount = rf24.getDynamicPayloadSize();

  if (inDataCount > 0)
  {
    rf24.read(rf24InData, inDataCount);

    for (int index = 0; index < inDataCount; index++)
    {
      if (rf24InData[index] == Orders::transStart)
      {
        if (rf24InData[index + 1] != Orders::requestEcho)
          HandleOrder(rf24InData + index, OrderSource::FromRF24);
      }
    }
  }
}

void Communication::StartESP8266()
{
  if (esp8266.kick())
    if (esp8266.setOprToSoftAP())
      if (esp8266.setSoftAPParam(esp8266SSID, esp8266PWD))
        if (esp8266.enableMUX())
          if (esp8266.startTCPServer(esp8266Port))
            if (esp8266.setTCPServerTimeout(0))
            {
              isESP8266Available = true;
              return;
            }
  isESP8266Available = false;
}

void Communication::UpdateESP8266()
{
  if (!isESP8266Available)
    return;

  int inDataCount = esp8266.recv(&esp8266ClientID, esp8266InData, inDataSize, 2);

  if (inDataCount > 0)
  {
    for (int index = 0; index < inDataCount; index++)
    {
      if (esp8266InData[index] == Orders::transStart)
        HandleOrder(esp8266InData + index, OrderSource::FromESP8266);
    }
  }
}

void Communication::HandleOrder(byte inData[], OrderSource orderSource)
{
  if (blockedOrder != 0)
    return;

  this->orderSource = orderSource;

  byte outData[outDataSize];
  byte outDataCounter = 0;

  outData[outDataCounter++] = Orders::transStart;

  if (inData[1] == Orders::requestEcho)
  {
    outData[outDataCounter++] = Orders::echo;
  }
  else if (inData[1] == Orders::requestSupplyVoltage)
  {
    float supplyVoltage = GetSupplyVoltage();
    outData[outDataCounter++] = Orders::supplyVoltage;
    outData[outDataCounter++] = (int)(supplyVoltage * 100) / 128;
    outData[outDataCounter++] = (int)(supplyVoltage * 100) % 128;
  }
  else if (inData[1] == Orders::requestChangeIO)
  {
    digitalWrite(pins[inData[2]], inData[3]);
    outData[outDataCounter++] = Orders::orderDone;
  }
  else if (inData[1] == Orders::requestMoveLeg)
  {
    robotAction.LegMoveToRelativelyDirectly(inData[2], Point(inData[3] - 64, inData[4] - 64, inData[5] - 64));
    outData[outDataCounter++] = Orders::orderDone;
  }
  else if (inData[1] == Orders::requestCalibrate)
  {
    robotAction.robot.CalibrateServos();
    outData[outDataCounter++] = Orders::orderDone;
  }
  else if (inData[1] >= 64 && inData[1] <= 108)
  {
    blockedOrder = inData[1];
    outData[outDataCounter++] = Orders::orderStart;
  }
  else if (inData[1] == Orders::requestMoveBody)
  {
    blockedOrder = inData[1];
    moveBodyParameters[0] = inData[2];
    moveBodyParameters[1] = inData[3];
    moveBodyParameters[2] = inData[4];
    outData[outDataCounter++] = Orders::orderStart;
  }
  else if (inData[1] == Orders::requestRotateBody)
  {
    blockedOrder = inData[1];
    rotateBodyParameters[0] = inData[2];
    rotateBodyParameters[1] = inData[3];
    rotateBodyParameters[2] = inData[4];
    outData[outDataCounter++] = Orders::orderStart;
  }
  else if (inData[1] == Orders::requestTwistBody)
  {
    blockedOrder = inData[1];
    twistBodyParameters[0] = inData[2];
    twistBodyParameters[1] = inData[3];
    twistBodyParameters[2] = inData[4];
    twistBodyParameters[3] = inData[5];
    twistBodyParameters[4] = inData[6];
    twistBodyParameters[5] = inData[7];
    outData[outDataCounter++] = Orders::orderStart;
  }
  outData[outDataCounter++] = Orders::transEnd;

  if (orderSource == OrderSource::FromSerial)
    Serial.write(outData, outDataCounter);
  else if (orderSource == OrderSource::FromESP8266)
    esp8266.send(esp8266ClientID, outData, outDataCounter);
}

void Communication::UpdateBlockedOrder()
{
  byte blockedOrder = this->blockedOrder;
  if (blockedOrder == 0)
    return;

  lastBlockedOrderTime = millis();

  orderState = OrderState::ExecuteStart;

  if (blockedOrder == Orders::requestCrawlForward)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.CrawlForward();
  }
  else if (blockedOrder == Orders::requestCrawlBackward)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.CrawlBackward();
  }
  else if (blockedOrder == Orders::requestTurnLeft)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.TurnLeft();
  }
  else if (blockedOrder == Orders::requestTurnRight)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.TurnRight();
  }
  else if (blockedOrder == Orders::requestActiveMode)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.ActiveMode();
  }
  else if (blockedOrder == Orders::requestSleepMode)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.SleepMode();
  }
  else if (blockedOrder == Orders::requestSwitchMode)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.SwitchMode();
  }
  else if (blockedOrder == Orders::requestInstallState)
  {
    SaveRobotBootState(Robot::State::Install);
    robotAction.robot.InstallState();
  }
  else if (blockedOrder == Orders::requestCalibrateState)
  {
    SaveRobotBootState(Robot::State::Calibrate);
    robotAction.robot.CalibrateState();
  }
  else if (blockedOrder == Orders::requestBootState)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.robot.BootState();
  }
  else if (blockedOrder == Orders::requestCalibrateVerify)
  {
    robotAction.robot.CalibrateVerify();
  }
  else if (blockedOrder == Orders::requestMoveBody)
  {
    SaveRobotBootState(Robot::State::Boot);
    float x = moveBodyParameters[0] - 64;
    float y = moveBodyParameters[1] - 64;
    float z = moveBodyParameters[2] - 64;
    robotAction.MoveBody(x, y, z);
  }
  else if (blockedOrder == Orders::requestRotateBody)
  {
    SaveRobotBootState(Robot::State::Boot);
    float x = rotateBodyParameters[0] - 64;
    float y = rotateBodyParameters[1] - 64;
    float z = rotateBodyParameters[2] - 64;
    robotAction.RotateBody(x, y, z);
  }
  else if (blockedOrder == Orders::requestTwistBody)
  {
    SaveRobotBootState(Robot::State::Boot);
    float xMove = twistBodyParameters[0] - 64;
    float yMove = twistBodyParameters[1] - 64;
    float zMove = twistBodyParameters[2] - 64;
    float xRotate = twistBodyParameters[3] - 64;
    float yRotate = twistBodyParameters[4] - 64;
    float zRotate = twistBodyParameters[5] - 64;
    robotAction.TwistBody(Point(xMove, yMove, zMove), Point(xRotate, yRotate, zRotate));
  }

  this->blockedOrder = 0;
  orderState = OrderState::ExecuteDone;
}

void Communication::CheckBlockedOrder()
{
  if (orderState != OrderState::ExecuteDone)
    return;

  byte outData[outDataSize];
  byte outDataCounter = 0;

  outData[outDataCounter++] = Orders::transStart;
  outData[outDataCounter++] = Orders::orderDone;
  outData[outDataCounter++] = Orders::transEnd;

  if (orderSource == OrderSource::FromSerial)
    Serial.write(outData, outDataCounter);
  else if (orderSource == OrderSource::FromESP8266)
    esp8266.send(esp8266ClientID, outData, outDataCounter);

  orderState = OrderState::ExecuteNone;
}

void Communication::UpdateAutoSleep()
{
  if (lastBlockedOrderTime != 0)
  {
    if (millis() - lastBlockedOrderTime > autoSleepOvertime)
    {
      if (robotAction.robot.state == Robot::State::Action)
        robotAction.SleepMode();
      lastBlockedOrderTime = 0;
    }
  }
}

void Communication::UpdateCommunication()
{
  UpdateStateLED();

  if (commFunction)
  {
    UpdateSerial();
    UpdateRF24();
    UpdateESP8266();
    CheckBlockedOrder();
  }
}

void Communication::UpdateOrder()
{
  UpdateBlockedOrder();
  UpdateAutoSleep();
}

void Communication::UpdateStateLED()
{
  if (ledCounter / ledBlinkCycle < abs(ledState))
  {
    if (ledCounter % ledBlinkCycle == 0)
      SetStateLed(ledState > 0 ? HIGH : LOW);
    else if (ledCounter % ledBlinkCycle == ledBlinkCycle / 2)
      SetStateLed(ledState > 0 ? LOW : HIGH);
  }

  ledCounter++;

  if (ledCounter / ledBlinkCycle >= abs(ledState) + 3)
  {
    ledCounter = 0;

    if (robotAction.robot.state == Robot::State::Action)
      ledState = 1;
    else if (robotAction.robot.state == Robot::State::Boot)
      ledState = 1;
    else if (robotAction.robot.state == Robot::State::Calibrate)
      ledState = 2;
    else if (robotAction.robot.state == Robot::State::Install)
      ledState = 3;

    if (robotAction.robot.power.powerGroupAutoSwitch)
      if (!robotAction.robot.power.powerGroupState)
        ledState = -1;
  }
}

float Communication::GetSupplyVoltage()
{
  return robotAction.robot.power.voltage;
}

void Communication::SaveRobotBootState(Robot::State state)
{
  byte stateByte = 0;

  if (state == Robot::State::Install)
    stateByte = 0;
  else if (state == Robot::State::Calibrate)
    stateByte = 1;
  else if (state == Robot::State::Boot)
    stateByte = 2;

  if (EEPROM.read(EepromAddresses::robotState) != stateByte)
    EEPROM.write(EepromAddresses::robotState, stateByte);
}

Robot::State Communication::GetRobotBootState()
{
  byte stateByte = EEPROM.read(EepromAddresses::robotState);
  Robot::State state;

  if (stateByte == 0)
    state = Robot::State::Install;
  else if (stateByte == 1)
    state = Robot::State::Calibrate;
  else if (stateByte == 2)
    state = Robot::State::Boot;

  return state;
}

void Communication::SetRobotBootState(Robot::State state)
{
  if (state == Robot::State::Install)
    robotAction.robot.InstallState();
  else if (state == Robot::State::Calibrate)
    robotAction.robot.CalibrateState();
  else if (state == Robot::State::Boot)
    robotAction.robot.BootState();
}

void UpdateService()
{
  sei();

  if (communication != NULL) {
    communication->robotAction.robot.Update();
    communication->UpdateCommunication();
  }
}

#endif
