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

#ifndef DFRobotIRPosition_cpp
#define DFRobotIRPosition_cpp

#include "stdint.h"

#define DFR_INVALID -1

class DFRobotIRPosition {

  const int IRAddress = 0xB0 >> 1; ///< IIC address of the sensor

  /*!
   *  @brief position data structure from IIC sensor
   */
  union PositionData {
    uint8_t receivedBuffer[16]; ///< received buffer for IIC read
    struct{
      uint8_t header;
      struct{
        uint8_t xLowByte; ///< position x low byte.
        uint8_t yLowByte; ///< position y low byte.
        uint8_t xyHighByte; ///< position x and y high byte.
      }__attribute__ ((packed)) rawPosition[4]; ///< 4 raw positions.
    }__attribute__ ((packed))positionFrame;
  }__attribute__ ((packed)) positionData;

  int positionX[4]; ///< position x.
  int positionY[4]; ///< position y.

  /*!
   *  @brief Write two byte into the sensor to initialize and send data.
   *
   *  @param first  the first byte
   *  @param second the second byte
   */
  void writeTwoIICByte(uint8_t first, uint8_t second);

public:

  /*!
   *  @brief Constructor.
   */
  DFRobotIRPosition();

  /*!
   *  @brief Destructor.
   */
  ~DFRobotIRPosition();

  /*!
   *  @brief Initialize the sensor.
   */
  void begin();

  /*!
   *  @brief Request the position.
   *         IIC will block until all the data is transmitted.
   */
  void requestPosition();

  /*!
   *  @brief After requesting the position, read the data from the sensor.
   *         IIC will block until all the data is read.
   *
   *  @return Whether data from the sensor is ready to be read.
   *  @retval true Is ready
   *  @retval false Is not ready
   */
  bool readPosition();

  /*!
   *  @brief Get the last read X position of the point.
   *
   *  @param index The index of the 4 light objects ranging from 0 to 3.
   *
   *  @return The X position corresponing to the index ranging from 0 to 1023.
   *          0 means LEFT from the sensor perspective.
   *          DFR_INVALID means empty or index is invalid.
   */
  int getX(int index);
  
  /*!
   *  @brief Get the last read Y position of the point.
   *
   *  @param index The index of the 4 light objects ranging from 0 to 3.
   *
   *  @return The Y position corresponing to the index ranging from 0 to 767.
   *          0 means TOP from the sensor perspective.
   *          DFR_INVALID means empty or index is invalid.
   */
  int getY(int index);
};

#endif
