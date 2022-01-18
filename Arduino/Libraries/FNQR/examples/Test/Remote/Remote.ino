/*
 * Sketch     Self diagnosis sketch for Remote
 * Platform   Freenove Smart Car Remote (Arduino/Genuino Uno) with 
 *            Freenove Smart Car Remote Shield and Freenove UNO
 * Brief      This sketch is used to diagnose the remote after it has been assembled.
 *            If your remote is not working properly, follow the steps below to diagnose and fix.
 * Steps      1. Install NRF24L01 module to the remote.
 *            2. Connect remote to computer via USB cable and choose the right board and port.
 *               Then open Serial Moniter with baud 115200.
 *            4. Upolad this sketch to the remote.
 *               The Serial Moniter will show diagnostic information. 
 *               Operate the remote and the Serial Moniter will show relevant information.
 *            5. Please check the diagnostic information and try to fix the problem.
 *               Then press the RESET button to run this sketch again to see if the problem has been fixed.
 *               If yes, please upload the default sketch again to verify if the remote is working properly.
 *               If no or you can't fix the problem, please send diagnostic information and how did the 
 *               remote behave to our support team (support@freenove.com).
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/11/15
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#ifndef ARDUINO_AVR_UNO
#error Wrong board. Please choose "Arduino/Genuino Uno"
#endif

#include <SPI.h>
#include "RF24.h"
#include <FlexiTimer2.h>

RF24 rf24(9, 10);

enum InputPin { Pot1, Pot2, JoystickX, JoystickY, JoystickZ, S1, S2, S3, None };

const int pot1Pin = A0,         // define POT1
          pot2Pin = A1,         // define POT2
          joystickXPin = A2,    // define pin for direction X of joystick
          joystickYPin = A3,    // define pin for direction Y of joystick
          joystickZPin = 7,     // define pin for direction Z of joystick
          s1Pin = 4,            // define pin for S1
          s2Pin = 3,            // define pin for S2
          s3Pin = 2,            // define pin for S3
          led1Pin = 6,          // define pin for LED1 which is close to POT1 and used to indicate the state of POT1
          led2Pin = 5,          // define pin for LED2 which is close to POT2 and used to indicate the state of POT2
          led3Pin = 8;          // define pin for LED3 which is close to NRF24L01 and used to indicate the state of NRF24L01

volatile int ledState = 1;

void setup() {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Freenove Smart Car Remote Shield and Freenove UNO");
  Serial.println("------------------------------------------------------------------------------------------");

  pinMode(joystickZPin, INPUT);
  pinMode(s1Pin, INPUT);
  pinMode(s2Pin, INPUT);
  pinMode(s2Pin, INPUT);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);

  FlexiTimer2::set(200, UpdateService);
  FlexiTimer2::start();

//pinMode(12, INPUT_PULLUP);
  if (rf24.begin()) {
    Serial.println("NRF24L01: OK.");
  }
  else {
    Serial.println("NRF24L01: NG. Module not inatalled/ Hardware failure");
    ledState = 2;
  }

  Serial.println("------------------------------------------------------------------------------------------");
  Serial.println("Please start to operate the remote...");
}

void loop() {
  int pot1Value = analogRead(pot1Pin);
  int pot2Value = analogRead(pot2Pin);
  int joystickXValue = analogRead(joystickXPin);
  int joystickYValue = analogRead(joystickYPin);
  bool joystickZValue = digitalRead(joystickZPin);
  bool s1Value = digitalRead(s1Pin);
  bool s2Value = digitalRead(s2Pin);
  bool s3Value = digitalRead(s3Pin);

  static int pot1ValueBefore = pot1Value;
  static int pot2ValueBefore = pot2Value;
  static int joystickXValueBefore = joystickXValue;
  static int joystickYValueBefore = joystickYValue;
  static bool joystickZValueBefore = joystickZValue;
  static bool s1ValueBefore = s1Value;
  static bool s2ValueBefore = s2Value;
  static bool s3ValueBefore = s3Value;

  static InputPin inputPin = InputPin::None;
  static InputPin inputPinBefore = inputPin;

  static int printCounter = 0;
  const int maxPrintCount = 15;

  const int potIgnoredLength = 64;

  if(abs(pot1Value - pot1ValueBefore) > potIgnoredLength) {
    if(inputPin != InputPin::Pot1) {
      inputPin = InputPin::Pot1;
      Serial.println("");
      Serial.print("Pot1: ");
    }
    pot1ValueBefore = pot1Value;
    Serial.print(pot1Value);
    Serial.print(", ");
    printCounter++;
  }

  if(abs(pot2Value - pot2ValueBefore) > potIgnoredLength) {
    if(inputPin != InputPin::Pot2) {
      inputPin = InputPin::Pot2;
      Serial.println("");
      Serial.print("Pot2: ");
    }
    pot2ValueBefore = pot2Value;
    Serial.print(pot2Value);
    Serial.print(", ");
    printCounter++;
  }

  if(abs(joystickXValue - joystickXValueBefore) > potIgnoredLength) {
    if(inputPin != InputPin::JoystickX) {
      inputPin = InputPin::JoystickX;
      Serial.println("");
      Serial.print("JoystickX: ");
    }
    joystickXValueBefore = joystickXValue;
    Serial.print(joystickXValue);
    Serial.print(", ");
    printCounter++;
  }

  if(abs(joystickYValue - joystickYValueBefore) > potIgnoredLength) {
    if(inputPin != InputPin::JoystickY) {
      inputPin = InputPin::JoystickY;
      Serial.println("");
      Serial.print("JoystickY: ");
    }
    joystickYValueBefore = joystickYValue;
    Serial.print(joystickYValue);
    Serial.print(", ");
    printCounter++;
  }

  if(joystickZValue != joystickZValueBefore) {
    delay(10);
    if(joystickZValue != joystickZValueBefore) {
      if(inputPin != InputPin::JoystickZ) {
        inputPin = InputPin::JoystickZ;
        Serial.println("");
        Serial.print("JoystickZ: ");
      }
      joystickZValueBefore = joystickZValue;
      Serial.print(joystickZValue);
      Serial.print(", ");
      printCounter++;
    }
  }

  if(s1Value != s1ValueBefore) {
    delay(10);
    if(s1Value != s1ValueBefore) {
      if(inputPin != InputPin::S1) {
        inputPin = InputPin::S1;
        Serial.println("");
        Serial.print("S1: ");
      }
      s1ValueBefore = s1Value;
      Serial.print(s1Value);
      Serial.print(", ");
      printCounter++;
    }
  }

  if(s2Value != s2ValueBefore) {
    delay(10);
    if(s2Value != s2ValueBefore) {
      if(inputPin != InputPin::S2) {
        inputPin = InputPin::S2;
        Serial.println("");
        Serial.print("S2: ");
      }
      s2ValueBefore = s2Value;
      Serial.print(s2Value);
      Serial.print(", ");
      printCounter++;
    }
  }

  if(s3Value != s3ValueBefore) {
    delay(10);
    if(s3Value != s3ValueBefore) {
      if(inputPin != InputPin::S3) {
        inputPin = InputPin::S3;
        Serial.println("");
        Serial.print("S3: ");
      }
      s3ValueBefore = s3Value;
      Serial.print(s3Value);
      Serial.print(", ");
      printCounter++;
    }
  }

  if(inputPin != inputPinBefore) {
    printCounter = 0;
  }
  inputPinBefore = inputPin;
  
  if(printCounter >= maxPrintCount) {
    Serial.println("");
    Serial.print("    ");
    printCounter = 0;
  }

  analogWrite(led1Pin, map(analogRead(pot1Pin), 0, 1023, 0, 255));
  analogWrite(led2Pin, map(analogRead(pot2Pin), 0, 1023, 0, 255));
}


void UpdateService()
{
  sei();

  UpdateStateLED();
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
  digitalWrite(led3Pin, state);
}
