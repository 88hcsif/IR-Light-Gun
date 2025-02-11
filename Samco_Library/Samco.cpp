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

#include "Samco.h"
#include "math.h"

bool Samco::track(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3) {
  // Remapping LED postions to use with library.
  positionXX[0] = x0;
  positionYY[0] = y0;
  positionXX[1] = x1;
  positionYY[1] = y1;
  positionXX[2] = x2;
  positionYY[2] = y2;
  positionXX[3] = x3;
  positionYY[3] = y3;

  // Wait for all positions to be recognised before starting
  if ((positionYY[0] != SAM_INVALID) && (positionYY[1] != SAM_INVALID) && (positionYY[2] != SAM_INVALID) && (positionYY[3] != SAM_INVALID)) {
    start = true;
  }

  if (start && ((positionYY[0] != SAM_INVALID) || (positionYY[1] != SAM_INVALID) || (positionYY[2] != SAM_INVALID) || (positionYY[3] != SAM_INVALID))) {
    for (int i = 0; i < 4; i++) {
      // Check if LED has been seen...
      if ( (positionYY[i] == SAM_INVALID) && (positionXX[i] == SAM_INVALID) ) {
        // if unseen make sure all quadrants have a value if missing apply value with buffer and set to unseen (this step is important for 1 LED usage)
        if (!(((positionY[0] < medianY) && (positionX[0] < medianX)) || ((positionY[1] < medianY) && (positionX[1] < medianX)) || ((positionY[2] < medianY) && (positionX[2] < medianX)) || ((positionY[3] < medianY) && (positionX[3] < medianX)))) {
          positionX[i] = medianX + (medianX - finalX[3]) - buff;
          positionY[i] = medianY + (medianY - finalY[3]) - buff;
          seen[0] = 0;
        }
        if (!(((positionY[0] < medianY) && (positionX[0] > medianX)) || ((positionY[1] < medianY) && (positionX[1] > medianX)) || ((positionY[2] < medianY) && (positionX[2] > medianX)) || ((positionY[3] < medianY) && (positionX[3] > medianX)))) {
          positionX[i] = medianX + (medianX - finalX[2]) + buff;
          positionY[i] = medianY + (medianY - finalY[2]) - buff;
          seen[1] = 0;
        }
        if (!(((positionY[0] > medianY) && (positionX[0] < medianX)) || ((positionY[1] > medianY) && (positionX[1] < medianX)) || ((positionY[2] > medianY) && (positionX[2] < medianX)) || ((positionY[3] > medianY) && (positionX[3] < medianX)))) {
          positionX[i] = medianX + (medianX - finalX[1]) - buff;
          positionY[i] = medianY + (medianY - finalY[1]) + buff;
          seen[2] = 0;
        }
        if (!(((positionY[0] > medianY) && (positionX[0] > medianX)) || ((positionY[1] > medianY) && (positionX[1] > medianX)) || ((positionY[2] > medianY) && (positionX[2] > medianX)) || ((positionY[3] > medianY) && (positionX[3] > medianX)))) {
          positionX[i] = medianX + (medianX - finalX[0]) + buff;
          positionY[i] = medianY + (medianY - finalY[0]) + buff;
          seen[3] = 0;
        }

        // if all quadrants have a value apply value with buffer and set to seen/unseen
        if (positionY[i] < medianY){
          if (positionX[i] < medianX){
            positionX[i] = medianX + (medianX - finalX[3]) - buff;
            positionY[i] = medianY + (medianY - finalY[3]) - buff;
            seen[0] = 0;
          }
        }
        if (positionY[i] < medianY){
          if (positionX[i] > medianX){
            positionX[i] = medianX + (medianX - finalX[2]) + buff;
            positionY[i] = medianY + (medianY - finalY[2]) - buff;
            seen[1] = 0;
          }
        }           
        if (positionY[i] > medianY){
          if (positionX[i] < medianX){
            positionX[i] = medianX + (medianX - finalX[1]) - buff;
            positionY[i] = medianY + (medianY - finalY[1]) + buff;
            seen[2] = 0;
          }
        }
        if (positionY[i] > medianY){
          if (positionX[i] > medianX){
            positionX[i] = medianX + (medianX - finalX[0]) + buff;
            positionY[i] = medianY + (medianY - finalY[0]) + buff;
            seen[3] = 0;
          }
        }
      } else {
        // If LEDS have been seen place in correct quadrant, apply buffer and set to seen.
        if (positionYY[i] < medianY){
          if (positionXX[i] < medianX){
            positionX[i] = positionXX[i] - buff;
            positionY[i] = positionYY[i] - buff;
            seen[0]++;
          } else if (positionXX[i] > medianX){
            positionX[i] = positionXX[i] + buff;
            positionY[i] = positionYY[i] - buff;
            seen[1]++;
          }
        } else if (positionYY[i] > medianY){
          if (positionXX[i] < medianX){
            positionX[i] = positionXX[i] - buff;
            positionY[i] = positionYY[i] + buff;
            seen[2]++;
          } else if (positionXX[i] > medianX){
            positionX[i] = positionXX[i] + buff;
            positionY[i] = positionYY[i] + buff;
            seen[3]++;
          }
        }
      }

      // Arrange all values in to quadrants and remove buffer.
      // If LEDS have been seen use thier value.
      // If LEDS haven't been seen work out values from live positions.
      if (positionY[i] < medianY) {
        if (positionX[i] < medianX) {
          if (seen[0] > 1) { 
            finalX[0] = positionX[i] + buff;
            finalY[0] = positionY[i] + buff;
          } else if (positionY[i] < 0) {
            finalX[0] = finalX[2] + (yDistLeft * sinf(angle));
            finalY[0] = finalY[2] - (yDistLeft * cosf(angle));
          } else if (positionX[i] < 0) {
            finalX[0] = finalX[1] - (xDistTop * cosf(angle));
            finalY[0] = finalY[1] - (xDistTop * sinf(angle));
          }
        } else if (positionX[i] > medianX) {
          if (seen[1] > 1) {
            finalX[1] = positionX[i] - buff;
            finalY[1] = positionY[i] + buff;
          } else if (positionY[i] < 0) {
            finalX[1] = finalX[3] + (yDistRight * sinf(angle));
            finalY[1] = finalY[3] - (yDistRight * cosf(angle));
          } else if (positionX[i] >= 1024) {
            finalX[1] = finalX[0] + (xDistTop * cosf(angle));
            finalY[1] = finalY[0] + (xDistTop * sinf(angle));
          }
        }
      } else if (positionY[i] > medianY) {
        if (positionX[i] < medianX) {
          if (seen[2] > 1) {
            finalX[2] = positionX[i] + buff;
            finalY[2] = positionY[i] - buff;
          } else if (positionY[i] >= 768) {
            finalX[2] = finalX[0] - (yDistLeft * sinf(angle));
            finalY[2] = finalY[0] + (yDistLeft * cosf(angle));
          } else if (positionX[i] < 0) {
            finalX[2] = finalX[3] - (xDistBottom * cosf(angle));
            finalY[2] = finalY[3] - (xDistBottom * sinf(angle));
          }
        } else if (positionX[i] > medianX) {
          if ((seen[3] > 1)) {
            finalX[3] = positionX[i] - buff;
            finalY[3] = positionY[i] - buff;
          } else if (positionY[i] >= 768) {
            finalX[3] = finalX[1] - (yDistRight * sinf(angle));
            finalY[3] = finalY[1] + (yDistRight * cosf(angle));
          } else if (positionX[i] >= 1024) {
            finalX[3] = finalX[2] + (xDistBottom * cosf(angle));
            finalY[3] = finalY[2] + (xDistBottom * sinf(angle));
          }
        }
      }
    }

    // If all LEDS can be seen update median (resets sketch stop hangs on glitches)
    if ((positionYY[0] != SAM_INVALID) && (positionYY[1] != SAM_INVALID) && (positionYY[2] != SAM_INVALID) && (positionYY[3] != SAM_INVALID)){
      medianY = (positionY[0] + positionY[1] + positionY[2] + positionY[3]) / 4;
      medianX = (positionX[0] + positionX[1] + positionX[2] + positionX[3]) / 4;
      if (yDistLeft > yDistRight){
        angleOffset = asinf((float)(yDistLeft-yDistRight)/(xDistTop)) / 2;
      } else if (yDistRight > yDistLeft){
        angleOffset = (asinf((float)(yDistRight-yDistLeft)/(xDistTop)) * -1) / 2;
      } else{
        angleOffset = 0;
      }
    } else{
      medianY = (finalY[0] + finalY[1] + finalY[2] + finalY[3]) / 4;
      medianX = (finalX[0] + finalX[1] + finalX[2] + finalX[3]) / 4;
    }

    // If 2 LEDS can be seen and loop has run through 5 times update angle and distances
    if ((seen[0] > 5) && (seen[2] > 5)) {
      angle = (atan2f(finalX[2] - finalX[0], finalY[2] - finalY[0]) * -1);
      yDistLeft = hypotf((finalY[0] - finalY[2]), (finalX[0] - finalX[2]));
    } else {
      yDistLeft = yDistLeft;
    }

    if ((seen[3] > 5) && (seen[1] > 5)) {
      angle = (atan2f(finalX[3] - finalX[1], finalY[3] - finalY[1]) * -1);
      yDistRight = hypotf((finalY[1] - finalY[3]), (finalX[1] - finalX[3]));
    } else {
      yDistRight = yDistRight;
    }

    if ((seen[0] > 5) && (seen[1] > 5)) {
      angle = (atan2f(finalX[1] - finalX[0], finalY[1] - finalY[0]) * -1) + 1.57 - angleOffset;
      xDistTop = hypotf((finalY[0] - finalY[1]), (finalX[0] - finalX[1]));
    } else {
      xDistTop = xDistTop;
    }

    if ((seen[3] > 5) && (seen[2] > 5)) {
      angle = (atan2f(finalX[3] - finalX[2], finalY[3] - finalY[2]) * -1) + 1.57 + angleOffset;
      xDistBottom = hypotf((finalY[3] - finalY[2]), (finalX[3] - finalX[2]));
    } else {
      xDistBottom = xDistBottom;
    }

    return true;
  }
  return false;
}

int Samco::getX(int index) {
  if (index < 0 || index > 3) return 0;
  return finalX[index];
}

int Samco::getY(int index) {
  if (index < 0 || index > 3) return 0;
  return finalY[index];
}
