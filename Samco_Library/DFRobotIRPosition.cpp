/*
 * Modified DFRobot's positioning ir camera library
 *
 * Copyright 2021 88hcsif
 *
 * Derived from DFRobot's positioning ir camera library:
 * Copyright: [DFRobot](http://www.dfrobot.com), 2016
 * Author:    [Angelo](Angelo.qiao@dfrobot.com)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "Arduino.h"
#include "DFRobotIRPosition.h"
#include "Wire.h"

DFRobotIRPosition::DFRobotIRPosition() {}

DFRobotIRPosition::~DFRobotIRPosition() {}

void DFRobotIRPosition::writeTwoIICByte(uint8_t first, uint8_t second) {
  Wire.beginTransmission(IRAddress);
  Wire.write(first);
  Wire.write(second);
  Wire.endTransmission();
}

void DFRobotIRPosition::begin() {
  Wire.begin();
  writeTwoIICByte(0x30,0x01);
  delay(10);
  writeTwoIICByte(0x30,0x08);
  delay(10);
  writeTwoIICByte(0x06,0x90);
  delay(10);
  writeTwoIICByte(0x08,0xC0);
  delay(10);
  writeTwoIICByte(0x1A,0x40);
  delay(10);
  writeTwoIICByte(0x33,0x33);
  delay(10);

  delay(100);
}

void DFRobotIRPosition::requestPosition() {
  Wire.beginTransmission(IRAddress);
  Wire.write(0x36);
  Wire.endTransmission();
  Wire.requestFrom(IRAddress, 16);
}

bool DFRobotIRPosition::readPosition() {
  if (Wire.available() == 16) { // read only if the data length is expected.
    for (int i=0; i<16; i++) {
      positionData.receivedBuffer[i] = Wire.read();
    }

    for (int i=0; i<4; i++) {
      positionX[i] = 1023 - ((uint16_t)(positionData.positionFrame.rawPosition[i].xLowByte)
      | ((uint16_t)(positionData.positionFrame.rawPosition[i].xyHighByte & 0x30U) << 4));

      positionY[i] = (uint16_t)(positionData.positionFrame.rawPosition[i].yLowByte)
      | ((uint16_t)(positionData.positionFrame.rawPosition[i].xyHighByte & 0xC0U) << 2);

      if (positionY[i] == 1023) {
        positionX[i] = DFR_INVALID;
        positionY[i] = DFR_INVALID;
      }
    }
    return true;
  } else { // otherwise skip them.
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }
}

int DFRobotIRPosition::getX(int index) {
  if (index < 0 || index > 3) return DFR_INVALID;
  return positionX[index];
}

int DFRobotIRPosition::getY(int index) {
  if (index < 0 || index > 3) return DFR_INVALID;
  return positionY[index];
}
