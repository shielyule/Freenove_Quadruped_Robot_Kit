/*
 * Sketch     Crawl function example
 * Platform   Freenove Quadruped Robot (Arduino/Genuino Mega 2560)
 * Brief      This sketch is used to show how to control Freenove Quadruped Robot with code.
 *            You can easily achieve custom function by using FNQR library we provide.
 * Note       Keep the power off when uploading the code to prevent the robot from suddenly moving 
 *            after the upload is complete. Before turning on the power, disconnect the USB cable 
 *            and place the robot on a smooth surface that has no other objects.
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/11/15
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#ifndef ARDUINO_AVR_MEGA2560
#error Wrong board. Please choose "Arduino/Genuino Mega or Mega 2560"
#endif

// Include FNQR (Freenove Quadruped Robot) library
#include <FNQR.h>

FNQR robot;

void setup() {
  // Custom setup code start

  // Custom setup code end
  // Start Freenove Quadruped Robot
  robot.Start();
}

void loop() {
  // Custom loop code start

  delay(2000);
  
  // Crawl forward
  robot.CrawlForward();
  robot.CrawlForward();
  robot.CrawlForward();
  robot.CrawlForward();
  delay(1000);

  // Crawl backward
  robot.CrawlBackward();
  robot.CrawlBackward();
  robot.CrawlBackward();
  robot.CrawlBackward();
  delay(1000);

  // Turn left
  robot.TurnLeft();
  robot.TurnLeft();
  robot.TurnLeft();
  robot.TurnLeft();
  delay(1000);

  // Turn right
  robot.TurnRight();
  robot.TurnRight();
  robot.TurnRight();
  robot.TurnRight();
  delay(1000);

  // Sleep mode
  robot.SleepMode();
  delay(1000);

  // Active mode
  robot.ActiveMode();
  delay(1000);

  // Switch between active and sleep mode
  robot.SwitchMode();

  while (true);
  // Custom loop code end
}
