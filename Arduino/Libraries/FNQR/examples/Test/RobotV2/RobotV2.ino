/*
 * Sketch     Self diagnosis sketch for Robot
 * Platform   Freenove Quadruped/Hexapod Robot (Arduino/Genuino Mega 2560) with 
 *            Freenove Crawling Robot Controller V2.X (X: 0~9) (Marked on the control board)
 * Brief      This sketch is used to diagnose the robot after it has been assembled.
 *            If your robot is not working properly, follow the steps below to diagnose and fix.
 * Steps      1. Install ESP8266 module and NRF24L01 module (if have) to the robot.
 *            2. Turn off the power switch and then install full charged batteries to the robot.
 *            3. Connect robot to computer via USB cable and choose the right board and port.
 *               Then open Serial Moniter with baud 115200.
 *            4. Hold the bottom of the robot to prevent the servos from suddenly turning.
 *               Open the power switch and upolad this sketch to the robot.
 *               The Serial Moniter will show diagnostic information and the servos will turn to 
 *               installation state and then rotate slowly.
 *            5. Please check the diagnostic information and try to fix the problem.
 *               Then press the RESET button to run this sketch again to see if the problem has been fixed.
 *               If yes, please upload the default sketch again to verify if the robot is working properly.
 *               If no or you can't fix the problem, please send diagnostic information and how did the 
 *               robot behave to our support team (support@freenove.com).
 * Note       When you hold the the robot, your hand should be far away from the movable range of the legs.
 *            If your hand is clamped or any servo is jammed, please turn off the power switch immediately.
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/11/15
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#ifndef ARDUINO_AVR_MEGA2560
#error Wrong board. Please choose "Arduino/Genuino Mega or Mega 2560"
#endif

#include <EEPROM.h>
#include <SPI.h>
#include "RF24.h"
#include "ESP8266.h"
#include <Servo.h>
#include <FlexiTimer2.h>

const int pins[] = { 15, 14, 2, 3, A1, A0, 21, 20 };

const int controllerVersionAddress = 0;
const int controllerVersion = 20;

RF24 rf24 = RF24(9, 53);

ESP8266 esp8266 = ESP8266(Serial2, 115200);

Servo servos[18];
const int servosPins[] = { 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39};
const int servosPowersPins[] = { A13, A14, A15 };

volatile int ledState = 1;

void setup() {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Freenove Crawling Robot Controller V2.X");
  Serial.println("------------------------------------------------------------------------------------------");

  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < 8; i++)
    pinMode(pins[i], OUTPUT);
  for (int i = 0; i < 4; i++)
    digitalWrite(pins[i], HIGH);
  for (int i = 4; i < 8; i++)
    digitalWrite(pins[i], LOW);

  FlexiTimer2::set(200, UpdateService);
  FlexiTimer2::start();

  analogReference(EXTERNAL);
  float batteryVoltage = analogRead(A7) * 2.5 * 32 / (32 + 6.2) / 1023 * (6.2 + 2) / 2;
  Serial.print("Battery voltage: ");
  Serial.print(batteryVoltage);
  Serial.print(" V. ");
  if (batteryVoltage > 6.5 && batteryVoltage < 8.5) {
    Serial.println("OK.");
  }
  else {
    Serial.println("NG. No batteries/ Low battery voltage/ Power not opened");
    ledState = 2;
  }

  analogReference(DEFAULT);
  float arefVoltage = analogRead(A6) * 5.0 / 1023;
  Serial.print("AREF voltage: ");
  Serial.print(arefVoltage);
  Serial.print(" V. ");
  if (arefVoltage > 2.4 && arefVoltage < 2.6) {
    Serial.println("OK.");
  }
  else {
    Serial.println("NG. Hardware failure");
    ledState = 3;
  }
  analogReference(EXTERNAL);

  Serial.print("Version data: ");
  int versionDataBefore = EEPROM.read(controllerVersionAddress);
  Serial.print("Before: ");
  Serial.print(versionDataBefore);
  Serial.print(", ");
  EEPROM.write(controllerVersionAddress, controllerVersion);
  int versionDataNow = EEPROM.read(controllerVersionAddress);
  Serial.print("Now: ");
  Serial.print(versionDataNow);
  Serial.print(". ");
  if (versionDataBefore == controllerVersion && versionDataNow == controllerVersion) {
    Serial.println("OK.");
  }
  else if (versionDataBefore != controllerVersion && versionDataNow == controllerVersion) {
    Serial.println("OK. Version data was wrong but has been fixed.");
  }
  else {
    Serial.println("NG. Hardware failure");
    ledState = 4;
  }

  pinMode(50, INPUT_PULLUP);
  if (rf24.begin()) {
    Serial.println("NRF24L01: OK.");
  }
  else {
    Serial.println("NRF24L01: NG. Module not inatalled/ Hardware failure");
    ledState = 5;
  }

  if (esp8266.kick()) {
    Serial.println("ESP8266: OK.");
  }
  else {
    Serial.println("ESP8266: NG. Module not inatalled/ Hardware failure");
    ledState = 6;
  }

  Serial.println("------------------------------------------------------------------------------------------");
  Serial.println("Power for servos are turnning on...");

  delay(1000);

  for (int i = 0; i < 18; i++) {
    servos[i].attach(servosPins[i]);
    servos[i].write(90);
  }

  for (int i = 0; i < 3; i++) {
    pinMode(servosPowersPins[i], OUTPUT);
    digitalWrite(servosPowersPins[i], HIGH);
    delay(200);
  }

  Serial.println("All the servos are starting to rotate slowly...");

  delay(1000);
}

void loop() {
  static int angle = 90;
  static bool dir = true;
  const int range = 30;

  if(dir) angle++;
  else angle--;

  if(angle == 90 + range / 2) dir = false;
  if(angle == 90 - range / 2) dir = true;

  for (int i = 0; i < 18; i++)
    servos[i].write(angle);

  float batteryVoltage = analogRead(A7) * 2.5 * 32 / (32 + 6.2) / 1023 * (6.2 + 2) / 2;
  if (batteryVoltage < 5.5) {
    Serial.println("------------------------------------------------------------------------------------------");
    Serial.print("Battery voltage: ");
    Serial.print(batteryVoltage);
    Serial.print(" V. ");
    Serial.println("NG. Batteries protected/ Low battery voltage/ Power closed");
    while(true);
  }

  delay(50);
}

void UpdateService()
{
  sei();

  static int counter = 0;

  for (int i = 0; i < 4; i++)
    digitalWrite(pins[i], HIGH);
  for (int i = 4; i < 8; i++)
    digitalWrite(pins[i], LOW);
  digitalWrite(pins[counter % 8], counter % 8 < 4 ? LOW : HIGH);

  UpdateStateLED();

  counter++;
}

void UpdateStateLED()
{
  const static int stepLength = 2;
  const static int intervalSteps = 3;
  static int ledState = ::ledState;
  static int counter = 0;

  if (counter / stepLength < abs(ledState))
  {
    if (counter % stepLength == 0)
      SetStateLed(ledState > 0 ? HIGH : LOW);
    else if (counter % stepLength == stepLength / 2)
      SetStateLed(ledState > 0 ? LOW : HIGH);
  }

  counter++;

  if (counter / stepLength >= abs(ledState) + intervalSteps)
  {
    ledState = ::ledState;
    counter = 0;
  }
}

void SetStateLed(bool state)
{
  digitalWrite(LED_BUILTIN, state);
}
