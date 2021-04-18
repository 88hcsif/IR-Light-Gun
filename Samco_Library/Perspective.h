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

#ifndef Perspective_h
#define Perspective_h

class Perspective {

private:
  // TODO: do not hardcode these values, make them easy to set

  // float monitorHeight = 36.7f;
  // float monitorWidth = 61.1f;
  // float sensorBarLength = 20.0f;
  // float bezelTop = 0.85f;
  // float bezelBottom = 2.4f;
  // float bezelLeft = 0.85f;
  // float bezelRight = 0.85f;

  float monitorHeight = 64.9f; // 50.0 screen
  float monitorWidth = 88.6f; // 88.6 screen
  float sensorBarLength = 20.0f;
  float bezelTop = 6.7f;
  float bezelBottom = 8.2f;
  float bezelLeft = 11.6f;
  float bezelRight = 13.0f;

  bool init = false;
  float srcmatrix[16];
  float dstmatrix[16];
  float warpmatrix[16];

  float srcX = 512.0f;
  float srcY = 384.0f;

  int dstX;
  int dstY;

public:
  void warp(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3);
  int getX();
  int getY();
};

#endif
