/*
 * Project center of camera to a perspective plane defined by four known points
 *
 * Copyright 2021 88hcsif
 *
 * Derived from Wiimote Whiteboard library:
 * Copyright (c) 2008 Stephane Duchesneau
 * by Stephane Duchesneau <stephane.duchesneau@gmail.com>
 * Ported from Johnny Lee's C# WiiWhiteboard project (Warper.cs file)
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

#include "Perspective.h"
#include "math.h"

void computeSquareToQuad(
      float* mat,
      float x0, float y0,
      float x1, float y1,
      float x2, float y2,
      float x3, float y3) {

  float dx1 = x1 - x2;
  float dy1 = y1 - y2;
  float dx2 = x3 - x2;
  float dy2 = y3 - y2;
  float sx = x0 - x1 + x2 - x3;
  float sy = y0 - y1 + y2 - y3;
  float g = (sx * dy2 - dx2 * sy) / (dx1 * dy2 - dx2 * dy1);
  float h = (dx1 * sy - sx * dy1) / (dx1 * dy2 - dx2 * dy1);
  float a = x1 - x0 + g * x1;
  float b = x3 - x0 + h * x3;
  float c = x0;
  float d = y1 - y0 + g * y1;
  float e = y3 - y0 + h * y3;
  float f = y0;

  mat[ 0] = a;
  mat[ 1] = d;
  mat[ 2] = 0.0f;
  mat[ 3] = g;
  mat[ 4] = b;
  mat[ 5] = e;
  mat[ 6] = 0.0f;
  mat[ 7] = h;
  mat[ 8] = 0.0f;
  mat[ 9] = 0.0f;
  mat[10] = 1.0f;
  mat[11] = 0.0f;
  mat[12] = c;
  mat[13] = f;
  mat[14] = 0.0f;
  mat[15] = 1.0f;
}

void computeQuadToSquare(
    float* mat,
    float x0, float y0,
    float x1, float y1,
    float x2, float y2,
    float x3, float y3) {
  computeSquareToQuad(mat, x0, y0, x1, y1, x2, y2, x3, y3);

  float a = mat[ 0];
  float d = mat[ 1];
  float g = mat[ 3];
  float b = mat[ 4];
  float e = mat[ 5];
  float h = mat[ 7];
  float c = mat[12];
  float f = mat[13];

  float A =     e - f * h;
  float B = c * h - b;
  float C = b * f - c * e;
  float D = f * g - d;
  float E =     a - c * g;
  float F = c * d - a * f;
  float G = d * h - e * g;
  float H = b * g - a * h;
  float I = a * e - b * d;
  float idet = 1.0 / (a * A + b * D + c * G);

  mat[ 0] = A * idet;
  mat[ 1] = D * idet;
  mat[ 2] = 0.0f;
  mat[ 3] = G * idet;

  mat[ 4] = B * idet;
  mat[ 5] = E * idet;
  mat[ 6] = 0.0f;
  mat[ 7] = H * idet;

  mat[ 8] = 0.0f;
  mat[ 9] = 0.0f;
  mat[10] = 1.0f;
  mat[11] = 0.0f;

  mat[12] = C * idet;
  mat[13] = F * idet;
  mat[14] = 0.0f;
  mat[15] = I * idet;
}

void multMats(float* a, float* b, float* res) {
  for (int r = 0; r < 4; r++) {
    int ri = r * 4;
    for (int c = 0; c < 4; c++) {
      res[ri + c] = (
        a[ri + 0] * b[c +  0] +
        a[ri + 1] * b[c +  4] +
        a[ri + 2] * b[c +  8] +
        a[ri + 3] * b[c + 12]);
    }
  }
}

void Perspective::warp(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3) {
  if (!init) {
    float dx0 = (monitorWidth - sensorBarLength) / 2.0f;
    float dx1 = dx0 + sensorBarLength;
    float dy2 = monitorHeight;
    computeSquareToQuad(dstmatrix, dx0, 0.0f, dx1, 0.0f, dx0, dy2, dx1, dy2);
    init = true;
  }
  computeQuadToSquare(srcmatrix, float(x0), float(y0), float(x1), float(y1), float(x2), float(y2), float(x3), float(y3));
  multMats(srcmatrix, dstmatrix, warpmatrix);
  float r0 = (srcX * warpmatrix[0] + srcY * warpmatrix[4] + warpmatrix[12]);
  float r1 = (srcX * warpmatrix[1] + srcY * warpmatrix[5] + warpmatrix[13]);
  float r3 = (srcX * warpmatrix[3] + srcY * warpmatrix[7] + warpmatrix[15]);
  dstX = floorf(1000.0f*(((r0/r3)-bezelLeft)/(monitorWidth-bezelLeft-bezelRight)));
  dstY = floorf(1000.0f*(((r1/r3)-bezelTop)/(monitorHeight-bezelTop-bezelBottom)));
}

int Perspective::getX() {
  return dstX;
}

int Perspective::getY() {
  return dstY;
}
