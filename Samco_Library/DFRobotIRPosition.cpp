/*!
 * Modified DFRobot's Positioning ir camera
 *
 * Copyright 2021 88hcsif
 *
 * Original copyright:
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [Angelo](Angelo.qiao@dfrobot.com)
 * @version  V1.0
 * @date  2016-02-17
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
