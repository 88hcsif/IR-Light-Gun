/*!
 * Modified Samco Light Gun library for 4 LED setup
 *
 * Copyright 2021 88hcsif
 *
 * Original copyright:
 * @copyright  [Samco](http://www.samco.co.nz), 2020
 * @copyright GNU Lesser General Public License
 *
 * @author [Sam Ballantyne](samuelballantyne@hotmail.com)
 * @version  V1.0
 * @date  2020
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

  int FinalX[4] = {400,623,400,623};
  int FinalY[4] = {200,200,568,568};

  int xDistTop;
  int xDistBottom;
  int yDistLeft;
  int yDistRight;

  float angle;
  float angleOffset;

  int xx;
  int yy;

  bool start = false;

public:
  bool track(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, int cx, int cy);
  int testX(int index);  
  int testY(int index);
  int testMedianX();
  int testMedianY();
  int X();
  int Y();
};

#endif
