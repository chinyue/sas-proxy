// Copyright 2018 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// SASProxy
// Forward Steering Anagle Sensor CAN messages from one CAN bus to another.
// chinyue 2016-04-01

#include <mcp_can.h>
#include <mcp_can_dfs.h>

//#define DEBUG
#ifdef DEBUG
#define DBG_PRINT(x) Serial.print(x)
#define DBG_PRINTHEX(x) Serial.print(x, HEX)
#define DBG_PRINTLN(x) Serial.println(x)
#define DBG_PRINTLNHEX(x) Serial.println(x, HEX)
#else
#define DBG_PRINT(x)
#define DBG_PRINTHEX(x)
#define DBG_PRINTLN(x)
#define DBG_PRINTLNHEX(x)
#endif


const byte READ_CS_PIN = 9;
const byte SEND_CS_PIN = 10;

MCP_CAN readCan(READ_CS_PIN);
MCP_CAN sendCan(SEND_CS_PIN);

// CAN ID of the Steering Angle Sensor.
const unsigned long CAN_ID_SAS = 0x2;

// It is observed that the first 2 bytes of SAS message is 0x800C after starting
// the car without moving steering wheel. And when turning the steering wheel,
// the first 2 bytes are
//   0x000E for center
//   0x001E for 1 degree left
//   0x002E for 2 degrees left
//   0xFFFE for 1 degree right
//   0xFFEE for 2 degrees right
// So looks like the higher 12 bits of the first 2 bytes are steering angle
// while the lower 4 bits are some kind of flags.
//
// The third byte of SAS message is checksum + serial. The lower nibble is
// serial, increases from 0 to F when sending message each time and then starts
// over again. The higher nibble is checksum, its calculation looks like:
// XOR the 4 nibbles of the first 2 bytes and serial.

const int MAX_ANGLE = 0xF800;

// Scaling factor to apply.
// 580 is the left turn lock position of stock steering gear and 440 is '15 STI.
const float SCALE_FACTOR = 580.0 / 440.0;


int get_angle(byte *data) {
  return (*(int *)data) >> 4;
}

void set_angle(byte *data, int angle) {
  int flags = (*(int *)data) & 0xF;
  *(int *)data = (angle << 4) + flags;
}

void recalc_checksum(byte *data) {
  byte nibble1, nibble2, nibble3, nibble4, serial, checksum;

  nibble1 = (data[0] >> 4) & 0xF;
  nibble2 = data[0] & 0xF;
  nibble3 = (data[1] >> 4) & 0xF;
  nibble4 = data[1] & 0xF;
  serial = data[2] & 0xF;
  checksum = (nibble1 ^ nibble2 ^ nibble3 ^ nibble4 ^ serial) & 0xF;
  data[2] = (checksum << 4) + serial;
}

void print_data(byte *data, byte len) {
  (void)data;

  for (int i = 0; i < len; ++i) {
    DBG_PRINT(" ");
    DBG_PRINTHEX(data[i]);
  }
  DBG_PRINTLN("");  
}

void setup() {
  Serial.begin(115200);

  while (readCan.begin(CAN_500KBPS) != CAN_OK) {
    Serial.println("Failed to init read CAN BUS shield, retry...");
    delay(100);
  }
  while (sendCan.begin(CAN_500KBPS) != CAN_OK) {
    Serial.println("Failed to init send CAN BUS shield, retry...");
    delay(100);
  }
  Serial.println("CAN BUS shield init done.");
  Serial.print("Steering angle scale factor: ");
  Serial.println(SCALE_FACTOR, 2);
}

void loop() {
  unsigned long canId;
  byte canExt, canRtr;
  byte data[8];
  byte len;
  int angle, scaledAngle;

  if (readCan.checkReceive() == CAN_MSGAVAIL) {
    readCan.readMsgBuf(&len, data);
    canId = readCan.getCanId();
    canExt = readCan.isExtendedFrame();
    canRtr = readCan.isRemoteRequest();

    if (canId != CAN_ID_SAS)
      return;

    DBG_PRINTLN("");
    DBG_PRINT("CAN ID: ");
    DBG_PRINTLNHEX(canId);
    DBG_PRINT("\tEXT: ");
    DBG_PRINTLNHEX(canExt);
    DBG_PRINT("\tRTR: ");
    DBG_PRINTLNHEX(canRtr);
    DBG_PRINT("\tDATA:");
    print_data(data, len);

    angle = get_angle(data);
    if (angle == MAX_ANGLE) {
      DBG_PRINTLN("\tSteering angle invalid, forward only.");
    } else {
      scaledAngle = angle * SCALE_FACTOR;
      DBG_PRINT("\tSteering Angle: ");
      DBG_PRINT(angle);
      DBG_PRINT(" -> ");
      DBG_PRINT(scaledAngle);
      DBG_PRINTLN("");

      set_angle(data, scaledAngle);
      recalc_checksum(data);
      DBG_PRINT("\tDATA:");
      print_data(data, len);
    }

    DBG_PRINT("Forwarding CAN message... ");
    if (sendCan.sendMsgBuf(canId, canExt, canRtr, len, data) == CAN_OK) {
      DBG_PRINTLN("done.");
    } else {
      DBG_PRINTLN("failed!");
    }
  }
}
