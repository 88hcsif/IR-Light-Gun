/*
 * Modified processing sketch to use with Samco_4IR_Test.ino
 *
 * Copyright 2021 88hcsif
 *
 * Derived from Samco Light Gun processing sketch:
 * Copyright: [Samco](https://github.com/samuelballantyne), April 2020
 * Author [Sam Ballantyne](samuelballantyne@hotmail.com)
 *
 * Projection code derived from Wiimote Whiteboard library:
 * Copyright (c) 2008 Stephane Duchesneau
 * by Stephane Duchesneau <stephane.duchesneau@gmail.com>
 * Ported from Johnny Lee's C# WiiWhiteboard project (Warper.cs file)
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

import processing.serial.*;

int BUFFER = 100;
int WIDTH = 1023;
int HEIGHT = 768;
int SCALE = 2;
int INVALID = 9999;
int INVALID_IN = -1;

int port = 9;

int lf = 10;  
String myString = null;
Serial myPort; 

class Point {
  int x = INVALID;
  int y = INVALID;
  color c;
  int s = 15;

  Point(color c) {
    this.c = c;
  }

  Point(color c, int s) {
    this.c = c;
    this.s = s;
  }

  void draw() {
    if (x != INVALID && y != INVALID) {
      ellipseMode(RADIUS);
      fill(c);
      strokeWeight(1);
      ellipse(x, y, s, s);
    }
  }

  void segment(Point p) {
    if (x != INVALID && y != INVALID && p.x != INVALID && p.y != INVALID) {
      strokeWeight(2);
      line(x, y, p.x, p.y);
    }
  }
}

void drawPolygon(Point ... points) {
  for (int i = 0; i < points.length; i++) {
    points[i].segment(points[(i+1)%points.length]);
  }
  for (int i = 0; i < points.length; i++) {
    points[i].draw();
  }
}

Point[] lights = new Point[4];
Point mouse = new Point(color(0xFF, 0xFF, 0x00));
Point[] monitor = new Point[4];
Point[] screen = new Point[4];
Point center = new Point(color(0xFF, 0xFF, 0x00), 5);

void settings() {
  size((WIDTH + 2*BUFFER)/SCALE, (HEIGHT + 2*BUFFER)/SCALE);
}

void setup() {
  // List all the available serial ports
  printArray(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[port], 9600);
  delay (100);
  myPort.clear();

  myString = myPort.readStringUntil('\n');
  myString = null;
  myPort.bufferUntil('\n');
  frameRate(60);

  center.x = (WIDTH/2+BUFFER)/SCALE;
  center.y = (HEIGHT/2+BUFFER)/SCALE;

  lights[0] = new Point(color(0xFF, 0x00, 0x00));
  lights[1] = new Point(color(0x00, 0xFF, 0x00));
  lights[2] = new Point(color(0xFF, 0xFF, 0xFF));
  lights[3] = new Point(color(0x00, 0x00, 0xFF));

  monitor[0] = new Point(color(0xFF, 0x00, 0x00), 5);
  monitor[1] = new Point(color(0x00, 0xFF, 0x00), 5);
  monitor[2] = new Point(color(0xFF, 0xFF, 0xFF), 5);
  monitor[3] = new Point(color(0x00, 0x00, 0xFF), 5);

  screen[0] = new Point(color(0xFF, 0x00, 0x00), 5);
  screen[1] = new Point(color(0x00, 0xFF, 0x00), 5);
  screen[2] = new Point(color(0xFF, 0xFF, 0xFF), 5);
  screen[3] = new Point(color(0x00, 0x00, 0xFF), 5);
}

void draw() {
  while(myPort.available() > 0) {
    myString = myPort.readStringUntil('\n');
    if(myString != null) {
      convertmyStringToCoordinates();

      background(0x5F, 0x5F, 0x5F);
      fill(0x7F, 0x7F, 0x7F);
      rect(BUFFER/SCALE, BUFFER/SCALE, WIDTH/SCALE, HEIGHT/SCALE);

      drawPolygon(monitor);
      drawPolygon(screen);
      drawPolygon(lights);
      center.draw();
      mouse.draw();
    }
  }
}

void convertmyStringToCoordinates() {
  try {
    // println(myString); 
    int[] output = int(trim(split(trim(myString), ','))); 

    for (int i = 0; i < 4; i++) {
      lights[i].x = output[2*i+0] == INVALID_IN ? INVALID : (output[2*i+0] + BUFFER)/SCALE;
      lights[i].y = output[2*i+1] == INVALID_IN ? INVALID : (output[2*i+1] + BUFFER)/SCALE;
    }
    lights[0].x = output[0] == INVALID_IN ? INVALID : (output[0] + BUFFER)/SCALE;
    lights[0].y = output[1] == INVALID_IN ? INVALID : (output[1] + BUFFER)/SCALE;
    lights[1].x = output[2] == INVALID_IN ? INVALID : (output[2] + BUFFER)/SCALE;
    lights[1].y = output[3] == INVALID_IN ? INVALID : (output[3] + BUFFER)/SCALE;
    lights[2].x = output[6] == INVALID_IN ? INVALID : (output[6] + BUFFER)/SCALE;
    lights[2].y = output[7] == INVALID_IN ? INVALID : (output[7] + BUFFER)/SCALE;
    lights[3].x = output[4] == INVALID_IN ? INVALID : (output[4] + BUFFER)/SCALE;
    lights[3].y = output[5] == INVALID_IN ? INVALID : (output[5] + BUFFER)/SCALE;

    mouse.x = ((int)(WIDTH  * ((float)output[8]/1000)) + BUFFER)/SCALE;
    mouse.y = ((int)(HEIGHT * ((float)output[9]/1000)) + BUFFER)/SCALE;

    warp(output[0], output[2], output[6], output[4], output[1], output[3], output[7], output[5]);
    for (int i = 0; i < 4; i++) {
      // print(String.format("%6d,%6d,", getX(i), getY(i)));
      monitor[i].x = (getX(i) + BUFFER)/SCALE;
      monitor[i].y = (getY(i) + BUFFER)/SCALE;
    }
    // println();
    for (int i = 4; i < 8; i++) {
      // print(String.format("%6d,%6d,", getX(i), getY(i)));
      screen[i-4].x = (getX(i) + BUFFER)/SCALE;
      screen[i-4].y = (getY(i) + BUFFER)/SCALE;
    }
    // println();
  } catch(ArrayIndexOutOfBoundsException exception) {
    println("ArrayIndexOutOfBoundsException"); 
  }
}

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

boolean init = false;
float[] srcmatrix = new float[16];
float[] dstmatrix = new float[16];
float[] warpmatrix = new float[16];

float[] srcX = new float[8];
float[] srcY = new float[8];

int[] dstX = new int[8];
int[] dstY = new int[8];

void computeSquareToQuad(
    float[] mat,
    float x0,
    float x1,
    float x2,
    float x3,
    float y0,
    float y1,
    float y2,
    float y3) {

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
    float[] mat,
    float x0,
    float x1,
    float x2,
    float x3,
    float y0,
    float y1,
    float y2,
    float y3) {
  computeSquareToQuad(mat, x0, x1, x2, x3, y0, y1, y2, y3);

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

void multMats(float[] a, float[] b, float[] res) {
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

void warp(
    float x0,
    float x1,
    float x2,
    float x3,
    float y0,
    float y1,
    float y2,
    float y3) {
  if (!init) {
    float ix0 = (monitorWidth - sensorBarLength) / 2.0f;
    float ix1 = ix0 + sensorBarLength;
    float iy2 = monitorHeight;
    computeQuadToSquare(srcmatrix, ix0, ix1, ix1, ix0, 0.0f, 0.0f, iy2, iy2);
    srcX[0] = 0.0f;
    srcY[0] = 0.0f;
    srcX[1] = monitorWidth;
    srcY[1] = 0.0f;
    srcX[2] = monitorWidth;
    srcY[2] = monitorHeight;
    srcX[3] = 0.0f;
    srcY[3] = monitorHeight;
    srcX[4] = bezelLeft;
    srcY[4] = bezelTop;
    srcX[5] = monitorWidth - bezelRight;
    srcY[5] = bezelTop;
    srcX[6] = monitorWidth - bezelRight;
    srcY[6] = monitorHeight - bezelBottom;
    srcX[7] = bezelLeft;
    srcY[7] = monitorHeight - bezelBottom;
    init = true;
  }
  computeSquareToQuad(dstmatrix, x0, x1, x2, x3, y0, y1, y2, y3);
  multMats(srcmatrix, dstmatrix, warpmatrix);
  for (int i = 0; i < 8; i++) {
    float r0 = (srcX[i] * warpmatrix[0] + srcY[i] * warpmatrix[4] + warpmatrix[12]);
    float r1 = (srcX[i] * warpmatrix[1] + srcY[i] * warpmatrix[5] + warpmatrix[13]);
    float r3 = (srcX[i] * warpmatrix[3] + srcY[i] * warpmatrix[7] + warpmatrix[15]);
    dstX[i] = (int)(r0/r3);
    dstY[i] = (int)(r1/r3);
  }
}

int getX(int i) {
  return dstX[i];
}

int getY(int i) {
  return dstY[i];
}
