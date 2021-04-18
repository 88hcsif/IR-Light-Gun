/*
 * Test sketch, see IR coordinates in Serial output
 *
 * Copyright 2021 88hcsif
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

#include <DFRobotIRPosition.h>
#include <Wire.h>

DFRobotIRPosition myDFRobotIRPosition;

void setup() {
  delay(500);
  myDFRobotIRPosition.begin();
  Serial.begin(9600);
}

void loop() {
  myDFRobotIRPosition.requestPosition();
  if (myDFRobotIRPosition.readPosition()) {
    for (int i = 0; i < 4; i++) {
      Serial.printf("%4d,%4d," , myDFRobotIRPosition.getX(i), myDFRobotIRPosition.getY(i));
    }
    Serial.println();
  } else {
    Serial.println("Device not available!");
  }
}
