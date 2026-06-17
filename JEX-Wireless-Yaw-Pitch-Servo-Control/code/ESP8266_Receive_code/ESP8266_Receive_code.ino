/*
  ==============================================================================
   Project       : ESP8266 Head Tracking with MPU6050 + Pan-Tilt Servos
   Version       : 1.0
   Board         : ESP8266 (NodeMCU / Wemos D1 Mini)
   File          : ESP8266_Receive_code.ino
   Author        : [JustElectronicX] (https://github.com/JustElectronicX)
   Created On    : [02-09-2025]
   Description   : Receives Yaw & Pitch data via ESP-NOW from sender ESP8266 
                   and maps it to two servo motors for pan/tilt movement.
   Comment       : - Pan servo attached on GPIO4 (D2)
                   - Tilt servo attached on GPIO5 (D1)
                   - Debugging available with DEBUG flag

   Status        : Working (tested with ESP8266 + SG90/MG90S servos)

   Purpose       :
     This firmware receives head tracking data from the sender module and drives
     two servos in a pan/tilt arrangement to replicate the MPU6050 orientation.

   Features:
     - ESP-NOW data reception
     - Mapping of yaw/pitch values to servo angles
     - Servo reset to neutral position at startup
     - Debugging support over Serial Monitor (baud: 115200)

   Supported Boards:
     - ESP8266 NodeMCU v1.0
     - Wemos D1 Mini

   Pin Usage (Receiver):
     - Pan Servo -> GPIO4 (D2)
     - Tilt Servo -> GPIO5 (D1)
     - External 5V power supply for servos required (shared GND)

   Dependencies:
     - ESP8266WiFi
     - espnow
     - Servo

  ------------------------------------------------------------------------------
  ==============================================================================
*/


#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>

Servo panServo, tiltServo;
#define SERVO_PIN_X 4  // panServo
#define SERVO_PIN_Y 5  // tiltServo
#define DEBUG 0 // Debugging flag 1= ON, 0 = OFF

volatile bool newDataAvailable = false;
// Structure to receive servo angles
struct Data_Packet {
  float pitch;
  float yaw;
};

Data_Packet myData;

void servoReset()
{
  panServo.write(20); // Initial position for panServo
  tiltServo.write(90); // Initial position for tiltServo
}

// Callback function for received data
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //servoX.write(receivedData.angleX);
  //servoY.write(receivedData.angleY);
  newDataAvailable = true;
  if (DEBUG) {
    //Serial.print("Received X Angle: "); Serial.print(receivedData.angleX);
    //Serial.print(" | Y Angle: "); Serial.println(receivedData.angleY);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  // Ensure STA mode for ESPNOW

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.println("ESP-NOW Initialized");

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // Attach servos to pins
  panServo.attach(SERVO_PIN_X, 600, 2400); //panServo
  tiltServo.attach(SERVO_PIN_Y, 600, 2400); //tiltServo
  servoReset();
  delay(3000);
}

void loop() {
  // Nothing needed in loop

  if (newDataAvailable) {
    // Process received data and move servos
    // Map and move servos based on pitch and yaw

    float angleX = constrain(myData.yaw, 1, 55);
    float angleY = constrain(myData.pitch, -90, 90);

    int panAngle = map(angleX, 1, 55, 20, 75);   // Adjust range as needed
    int tiltAngle = map(angleY, -90, 90, 180, 0); // Adjust range as needed

    panServo.write(panAngle);
    tiltServo.write(tiltAngle);

    newDataAvailable = false;
    if (DEBUG) {
      Serial.print("raw_X: ");
      Serial.print(myData.yaw);
      Serial.print(" | angle_X: ");
      Serial.print(angleX);
      Serial.print(" | Pan: ");
      Serial.print(panAngle);
      Serial.print(" | Tilt: ");
      Serial.println(tiltAngle);
    }
  }
}
