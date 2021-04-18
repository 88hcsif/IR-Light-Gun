/*
 * Modified Samco Light Gun library for 4 LED setup
 *
 * Copyright 2021 88hcsif
 *
 * Derived from Samco Light Gun library for 4 LED setup:
 * Copyright: [Samco](https://github.com/samuelballantyne), April 2020
 * Author [Sam Ballantyne](samuelballantyne@hotmail.com)
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

#ifndef Samco_h
#define Samco_h

#include "stdbool.h"
#include "DFRobotIRPosition.h"

#define SAM_INVALID DFR_INVALID

class Samco {

private:
  int positionXX[4];   ///< position x.
  int positionYY[4];   ///< position y.

  int positionX[4]; 
  int positionY[4];

  int seen[4];
  int buff = 50;

  int medianY = 768 / 2;
  int medianX = 1024 / 2;

  int finalX[4] = {400,624,400,624};
  int finalY[4] = {200,200,568,568};

  int xDistTop;
  int xDistBottom;
  int yDistLeft;
  int yDistRight;

  float angle;
  float angleOffset;

  bool start = false;

public:
  bool track(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3);
  int getX(int index);  
  int getY(int index);
};

#endif
