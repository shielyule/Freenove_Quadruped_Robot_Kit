/*
 * Sketch     Modify wireless communication channel of the remote
 * Platform   Freenove Smart Car Remote (Arduino/Genuino Uno)
 * Brief      This sketch is used to show how to modify wireless communication channel between robot 
 *            and remote when using default function.
 *            The robot should set the same channel to be able to controlled by romote.
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/11/15
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#ifndef ARDUINO_AVR_UNO
#error Wrong board. Please choose "Arduino/Genuino Uno"
#endif

// Include FNQR (Freenove Quadruped Robot) library
#include <FNQR.h>

FNQRRemote remote;

void setup() {
  // Set wireless communication channel
  // Call this function before remote.Start()
  // Set 5 byte type data
  remote.SetRC(1,2,3,4,5);
  // Start remote
  remote.Start();
}

void loop() {
  // Update remote
  remote.Update();
}
