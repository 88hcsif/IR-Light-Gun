/*
 * Test sketch, see button presses in Serial output
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

#include <Wire.h>

#define DEBOUNCE_DELAY 5

int currState[11];
int prevState[11];
int lastReportedState[11];
unsigned long lastDebounceTime[11];
int pinMap[] = {4, 7, 11, 9, 10, 12, A1, A0, A2, A3, A5};

#define MAX_STRING_SIZE 10
char labelMap[][MAX_STRING_SIZE] = {
  "PEDAL",
  "TRIGGER",
  "UP",
  "DOWN",
  "LEFT",
  "RIGHT",
  "A",
  "B",
  "START",
  "SELECT",
  "RELOAD"
};

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 11; i++) {
    pinMode(pinMap[i], INPUT_PULLUP);
    currState[i] = HIGH;
    prevState[i] = HIGH;
    lastReportedState[i] = HIGH;
  }

  delay(500);
}


void loop() {
  unsigned long now = millis();
  for (int i = 0; i < 11; i++) {
    currState[i] = digitalRead(pinMap[i]);
    if (currState[i] != lastReportedState[i]) {
      if (currState[i] != prevState[i]) {
        // reset the debouncing timer
        lastDebounceTime[i] = now;
      }
      if ((now - lastDebounceTime[i]) >= DEBOUNCE_DELAY) {
        Serial.printf("%-7s", labelMap[i]);
        if (currState[i] == LOW) {
          Serial.println(" PRESSED");
        } else {
          Serial.println(" RELEASED");
        }
        lastReportedState[i] = currState[i];
      }
    }
    prevState[i] = currState[i];
  }
}
