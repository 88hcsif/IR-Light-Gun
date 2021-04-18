/*
 * Test sketch to use with Samco_4IR_Processing_Sketch.pde
 *
 * Copyright 2021 88hcsif
 *
 * MODIFICATION HISTORY:
 * modified for use with new SAMCO libraries by 88hcsif, 2021.
 * modified for use with SAMCO light guns 4 Led setup by Sam Ballantyne April 2020.
 * modified for http://DFRobot.com by kurakura, Jan 2015.
 * modified for http://DFRobot.com by Lumi, Jan. 2014
 * modified output for Wii-BlobTrack program by RobotFreak http://www.letsmakerobots.com/user/1433
 * Wii Remote IR sensor test sample code by kako http://www.kako.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <Arduino.h>
#include <DFRobotIRPosition.h>
#include <Perspective.h>
#include <Samco.h>
#include <Wire.h>

DFRobotIRPosition myDFRobotIRPosition;
Perspective per;
Samco mySamco;

int rawX[4];
int rawY[4];

int finalX[4];
int finalY[4];

void setup() {
  delay(500);
  myDFRobotIRPosition.begin();
  Serial.begin(9600);
}

void loop() {
  if (getRawPosition()) {
    if (getEstimatePosition()) {
      printResults();
    }
  }
}

void printResults() {
  // for (int i = 0; i < 4; i++) {
  // Serial.printf("%5d,%5d," , rawX[i], rawY[i]);
  // }
  for (int i = 0; i < 4; i++) {
    Serial.printf("%5d,%5d," , finalX[i], finalY[i]);
  }
  Serial.printf("%5d,%5d," , per.getX(), per.getY());
  Serial.println();
}

bool getRawPosition() {
  myDFRobotIRPosition.requestPosition();
  if (myDFRobotIRPosition.readPosition()) {
    for (int i = 0; i < 4; i++) {
      rawX[i] = myDFRobotIRPosition.getX(i);
      rawY[i] = myDFRobotIRPosition.getY(i);
    }
    return true;
  } else {
    Serial.println("Device not available!");
    return false;
  }
}

bool getEstimatePosition() {
  if (mySamco.track(rawX[0], rawY[0], rawX[1], rawY[1], rawX[2], rawY[2], rawX[3], rawY[3])) {
    for (int i = 0; i < 4; i++) {
      finalX[i] = mySamco.getX(i);
      finalY[i] = mySamco.getY(i);
    }
    per.warp(finalX[0], finalY[0], finalX[1], finalY[1], finalX[2], finalY[2], finalX[3], finalY[3]);
    return true;
  } else {
    return false;
  }
}

