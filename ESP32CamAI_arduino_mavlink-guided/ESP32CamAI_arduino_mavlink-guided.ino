/*
  BSD 2-Clause License
  Copyright (c) 2020, ANM-P4F
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/common/mavlink_msg_set_attitude_target.h"

#include <WebSocketsServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "camera_wrap.h"
#include <vector>
//#define DEBUG
// #define SAVE_IMG

enum TRACK {
  TRACK_NONE = 0,
  TRACK_FW,
  TRACK_LEFT,
  TRACK_RIGHT,
  TRACK_STOP
};

const char* ssid = "networkname";    // <<< change this as yours
const char* password = "password"; // <<< change this as yours
//holds the current upload
int cameraInitState = -1;
uint8_t* jpgBuff = new uint8_t[68123];
size_t   jpgLength = 0;
uint8_t camNo = 0;
bool clientConnected = false;

//Creating UDP Listener Object.
WiFiUDP UDPServer;
IPAddress addrRemote;
unsigned int portRemote;
unsigned int UDPPort = 6868;
const int RECVLENGTH = 16;
byte packetBuffer[RECVLENGTH];

WebSocketsServer webSocket = WebSocketsServer(86);
String html_home;

const int LED_BUILT_IN        = 4;
const uint8_t TRACK_DUTY      = 100;
const int PIN_SERVO_PITCH     = 12;
// const int PIN_SERVO_YAW       = 2;
const int PINDC_LEFT_BACK     = 13;
const int PINDC_LEFT_FORWARD  = 15;
const int PINDC_RIGHT_BACK    = 14;
const int PINDC_RIGHT_FORWARD = 2;
const int LEFT_CHANNEL        = 2;
const int RIGHT_CHANNEL       = 3;
const int SERVO_PITCH_CHANNEL = 4;
const int SERVO_YAW_CHANNEL   = 5;
const int SERVO_RESOLUTION    = 16;
unsigned long previousMillisServo = 0;
const unsigned long intervalServo = 10;
bool servoUp = false;
bool servoDown = false;
bool servoRotateLeft = false;
bool servoRotateRight = false;
int posServo = 75;
int PWMTrackHIGH = 138;
int PWMTrackLOW = 138;

void servoWrite(uint8_t channel, uint8_t angle) {
  // regarding the datasheet of sg90 servo, pwm period is 20 ms and duty is 1->2ms
  uint32_t maxDuty = (pow(2, SERVO_RESOLUTION) - 1) / 10;
  uint32_t minDuty = (pow(2, SERVO_RESOLUTION) - 1) / 20;
  uint32_t duty = (maxDuty - minDuty) * angle / 180 + minDuty;
  ledcWrite(channel, duty);
}

void controlServo() {
  if (servoUp) {
    if (posServo > 2) {
      posServo -= 2;
    }
  }
  if (servoDown) {
    if (posServo < 180) {
      posServo += 2;
    }
  }
  servoWrite(SERVO_PITCH_CHANNEL, posServo);
}




void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      camNo = num;
      clientConnected = false;
      break;
    case WStype_CONNECTED:
      Serial.printf("[%u] Connected!\n", num);
      clientConnected = true;
      break;
    case WStype_TEXT:
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      Serial.println(type);
      break;
  }
}

std::vector<String> splitString(String data, String delimiter) {
  std::vector<String> ret;
  // initialize first part (string, delimiter)
  char* ptr = strtok((char*)data.c_str(), delimiter.c_str());

  while (ptr != NULL) {
    ret.push_back(String(ptr));
    // create next part
    ptr = strtok(NULL, delimiter.c_str());
  }
  return ret;
}

void processUDPData() {
  int cb = UDPServer.parsePacket();

  if (cb) {
    UDPServer.read(packetBuffer, RECVLENGTH);
    addrRemote = UDPServer.remoteIP();
    portRemote = UDPServer.remotePort();

    String strPackage = String((const char*)packetBuffer);
#ifdef DEBUG
    Serial.print("receive: ");
    // for (int y = 0; y < RECVLENGTH; y++){
    //   Serial.print(packetBuffer[y]);
    //   Serial.print("\n");
    // }
    Serial.print(strPackage);
    Serial.print(" from: ");
    Serial.print(addrRemote);
    Serial.print(":");
    Serial.println(portRemote);
#endif
    if (strPackage.equals("whoami")) {
      UDPServer.beginPacket(addrRemote, portRemote - 1);
      String res = "ESP32-CAM";
      UDPServer.write((const uint8_t*)res.c_str(), res.length());
      UDPServer.endPacket();
      Serial.println("response");
    } else if (strPackage.equals("forward")) {
      command_forward();
    } else if (strPackage.equals("backward")) {
      command_backward();
    } else if (strPackage.equals("left")) {
      command_left();
    } else if (strPackage.equals("right")) {
      command_right();
    } else if (strPackage.equals("stop")) {
      command_stop();

    } else if (strPackage.equals("camup")) {
      servoUp = true;
    } else if (strPackage.equals("camdown")) {
      servoDown = true;
    } else if (strPackage.equals("camstill")) {
      servoUp = false;
      servoDown = false;
    } else if (strPackage.equals("ledon")) {
      digitalWrite(LED_BUILT_IN, HIGH);
    } else if (strPackage.equals("ledoff")) {
      digitalWrite(LED_BUILT_IN, LOW);
    } else if (strPackage.equals("lefttrack")) {
    // command_lefttrack();
    } else if (strPackage.equals("righttrack")) {
    // command_righttrack();
    } else if (strPackage.equals("fwtrack")) {
   //  command_fwtrack();
    }

    memset(packetBuffer, 0, RECVLENGTH);
  }

}

void command_heartbeat() {

  //< ID 1 for this system
  int sysid = 1;
  //< The component sending the message.
  int compid = MAV_COMP_ID_PATHPLANNER;

  // Define the system type, in this case ground control station
  uint8_t system_type = MAV_TYPE_GCS;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

  uint8_t system_mode = 0;
  uint32_t custom_mode = 0;
  uint8_t system_state = 0;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message
  //delay(1);
  Serial.write(buf, len);
}



void command_forward() {

  //MAVLINK  MESSAGE
  int sysid = 1;
  int compid = MAV_COMP_ID_PATHPLANNER;
  uint32_t time_boot_ms = 0;
  uint8_t target_system = 1; /*<System ID of vehicle*/
  uint8_t target_component = 0; /*< Component ID of flight controller or just 0*/
  uint8_t type_mask = 163; /*  Use Yaw Rate + Throttle : 0b10100011 / 0xA3 / 163 (decimal)   Use Attitude + Throttle: 0b00100111 / 0x27 / 39 (decimal)*/
  float q = (1000); /*< Attitude quaternion (w, x, y, z order, zero-rotation is {1, 0, 0, 0})Note that zero-rotation causes vehicle to point North. */
  float body_roll_rate = 0; /*< Body roll rate not supported*/
  float body_pitch_rate = 1; /*< Body pitch rate not supporte*/
  float body_yaw_rate = 0.0; /*(Body yaw rate in radians*/
  float thrust = 1; /*< 0=throttle 0%, +1=forward at WP_SPEED, -1=backwards at WP_SPEED*/
  float thrust_body = (000);

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_attitude_target_pack(sysid, compid, &msg, time_boot_ms,  target_system, target_component, type_mask, 0000, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, 000);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  Serial.write(buf, len);
}



void command_backward() {


  //MAVLINK  MESSAGE
  int sysid = 1;
  int compid = MAV_COMP_ID_PATHPLANNER;
  uint32_t time_boot_ms = 0;
  uint8_t target_system = 1; /*<System ID of vehicle*/
  uint8_t target_component = 0; /*< Component ID of flight controller or just 0*/
  uint8_t type_mask = 163; /*  Use Yaw Rate + Throttle : 0b10100011 / 0xA3 / 163 (decimal)   Use Attitude + Throttle: 0b00100111 / 0x27 / 39 (decimal)*/
  float q = (1000); /*< Attitude quaternion (w, x, y, z order, zero-rotation is {1, 0, 0, 0})Note that zero-rotation causes vehicle to point North. */
  float body_roll_rate = 0; /*< Body roll rate not supported*/
  float body_pitch_rate = 1; /*< Body pitch rate not supporte*/
  float body_yaw_rate = 0.0; /*(Body yaw rate in radians*/
  float thrust = -1; /*< 0=throttle 0%, +1=forward at WP_SPEED, -1=backwards at WP_SPEED*/
  float thrust_body = (000);

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_attitude_target_pack(sysid, compid, &msg, time_boot_ms,  target_system, target_component, type_mask, 0000, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, 000);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  Serial.write(buf, len);
}



void command_left() {

  //MAVLINK  MESSAGE
  int sysid = 1;
  int compid = MAV_COMP_ID_PATHPLANNER;
  uint32_t time_boot_ms = 0;
  uint8_t target_system = 1; /*<System ID of vehicle*/
  uint8_t target_component = 0; /*< Component ID of flight controller or just 0*/
  uint8_t type_mask = 163; /*  Use Yaw Rate + Throttle : 0b10100011 / 0xA3 / 163 (decimal)   Use Attitude + Throttle: 0b00100111 / 0x27 / 39 (decimal)*/
  float q = (1000); /*< Attitude quaternion (w, x, y, z order, zero-rotation is {1, 0, 0, 0})Note that zero-rotation causes vehicle to point North. */
  float body_roll_rate = 0; /*< Body roll rate not supported*/
  float body_pitch_rate = 1; /*< Body pitch rate not supporte*/
  float body_yaw_rate = -0.2; /*(Body yaw rate in radians*/
  float thrust = 0.5; /*< 0=throttle 0%, +1=forward at WP_SPEED, -1=backwards at WP_SPEED*/
  float thrust_body = (000);

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_attitude_target_pack(sysid, compid, &msg, time_boot_ms,  target_system, target_component, type_mask, 0000, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, 000);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  Serial.write(buf, len);
}





void command_right() {

  //MAVLINK  MESSAGE
  int sysid = 1;
  int compid = MAV_COMP_ID_PATHPLANNER;
  uint32_t time_boot_ms = 0;
  uint8_t target_system = 1; /*<System ID of vehicle*/
  uint8_t target_component = 0; /*< Component ID of flight controller or just 0*/
  uint8_t type_mask = 163; /*  Use Yaw Rate + Throttle : 0b10100011 / 0xA3 / 163 (decimal)   Use Attitude + Throttle: 0b00100111 / 0x27 / 39 (decimal)*/
  float q = (1000); /*< Attitude quaternion (w, x, y, z order, zero-rotation is {1, 0, 0, 0})Note that zero-rotation causes vehicle to point North. */
  float body_roll_rate = 0; /*< Body roll rate not supported*/
  float body_pitch_rate = 1; /*< Body pitch rate not supporte*/
  float body_yaw_rate = 0.2; /*(Body yaw rate in radians*/
  float thrust = 0.5; /*< 0=throttle 0%, +1=forward at WP_SPEED, -1=backwards at WP_SPEED*/
  float thrust_body = (000);

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_attitude_target_pack(sysid, compid, &msg, time_boot_ms,  target_system, target_component, type_mask, 0000, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, 000);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  Serial.write(buf, len);
}



void command_stop() {


  //MAVLINK  MESSAGE
  int sysid = 1;
  int compid = MAV_COMP_ID_PATHPLANNER;
  uint32_t time_boot_ms = 0;
  uint8_t target_system = 1; /*<System ID of vehicle*/
  uint8_t target_component = 0; /*< Component ID of flight controller or just 0*/
  uint8_t type_mask = 163; /*  Use Yaw Rate + Throttle : 0b10100011 / 0xA3 / 163 (decimal)   Use Attitude + Throttle: 0b00100111 / 0x27 / 39 (decimal)*/
  float q = (1000); /*< Attitude quaternion (w, x, y, z order, zero-rotation is {1, 0, 0, 0})Note that zero-rotation causes vehicle to point North. */
  float body_roll_rate = 0; /*< Body roll rate not supported*/
  float body_pitch_rate = 1; /*< Body pitch rate not supporte*/
  float body_yaw_rate = 0.0; /*(Body yaw rate in radians*/
  float thrust = 1; /*< 0=throttle 0%, +1=forward at WP_SPEED, -1=backwards at WP_SPEED*/
  float thrust_body = (000);

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_attitude_target_pack(sysid, compid, &msg, time_boot_ms,  target_system, target_component, type_mask, 0000, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, 000);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  Serial.write(buf, len);
}



void setup(void) {

  Serial.begin(115200);
  Serial.print("\n");
#ifdef DEBUG
  Serial.setDebugOutput(true);
#endif

  pinMode(LED_BUILT_IN, OUTPUT);
  digitalWrite(LED_BUILT_IN, LOW);

  // 1. 50hz ==> period = 20ms (sg90 servo require 20ms pulse, duty cycle is 1->2ms: -90=>90degree)
  // 2. resolution = 16, maximum value is 2^16-1=65535
  // From 1 and 2 => -90=>90 degree or 0=>180degree ~ 3276=>6553
  ledcSetup(SERVO_PITCH_CHANNEL, 50, 16);//channel, freq, resolution
  ledcAttachPin(PIN_SERVO_PITCH, SERVO_PITCH_CHANNEL);// pin, channel
  servoWrite(SERVO_PITCH_CHANNEL, posServo);

  // ledcSetup(SERVO_YAW_CHANNEL, 50, 16);//channel, freq, resolution
  // ledcAttachPin(PIN_SERVO_YAW, SERVO_YAW_CHANNEL);// pin, channel
  // servoWrite(SERVO_YAW_CHANNEL, posServo);

  cameraInitState = initCamera();

  Serial.printf("camera init state %d\n", cameraInitState);

  if (cameraInitState != 0) {
    return;
  }

  //WIFI INIT
  Serial.printf("Connecting to %s\n", ssid);
  if (String(WiFi.SSID()) != String(ssid)) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  String ipAddress = WiFi.localIP().toString();;
  Serial.println(ipAddress);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  UDPServer.begin(UDPPort);
}

void loop(void) {
  webSocket.loop();
  if (clientConnected == true) {
    grabImage(jpgLength, jpgBuff);
    webSocket.sendBIN(camNo, jpgBuff, jpgLength);
    // Serial.print("send img: ");
    // Serial.println(jpgLength);
    command_heartbeat();
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisServo >= intervalServo) {
    previousMillisServo = currentMillis;

    processUDPData();
    controlServo();


  }

#ifdef DEBUG
  if (Serial.available()) {
    String data = Serial.readString();
    Serial.println(data);
    std::vector<String> vposVals = splitString(data, ",");
    if (vposVals.size() != 4) {
      return;
    }
    int left0 = vposVals[0].toInt();
    int left1 = vposVals[1].toInt();
    int left2 = vposVals[2].toInt();
    int left3 = vposVals[3].toInt();

  }
#endif
}
