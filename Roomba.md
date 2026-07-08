[![LiaScript](https://github.com/LiaScript/LiaScript/blob/ffe73bc438077d2d5a2e6a755ffe1e445449f9d5/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/AetherHey008/ENGCSRB-Roomba/refs/heads/main/Roomba.md)
<!--
author: Leonie Blum, Matteo Poma
title: Final Projekt Presentation at "Englisch Einführung in die Fachsprache"
-->

# Final Projekt Presentation: Roomba (Alvik)



## Who is Roomba? 

![](https://eu.robotshop.com/cdn/shop/files/arduino-alvik-robotics-learning-tool-w-nano-esp32-micropython-educational-01img_1200x1200.webp?v=1720470344)

**<u> Hardware </u>**

**Controllers:**
- 32-bit microcontroller (Nano ESP32)
- STM32F411RC
- 2 DC motors with wheel encoders - up to 13 cm/s
- Rechargeable Li-ion battery (18650 battery)
- USB-C interface (for programming and charging)

**Sensors:**
- 7x Capacitive Touch sensor
- 6-axis Gyro Accelerometer
- 5 line-tracking sensors
- 3x Line Follower array 
- Distance Time-to-Flight sensor - up to 350cm
- RGB Color detection
  
**Programming:**
- Arduino IDE (C++)
![](https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcS3auC2CpU6I-yglDVBGB9YwpkkEQV2STe9W_zEOSF1Zw&s=10)



---

## What is Roombas Mission
**<u> Mission 1: Object follower </u>**

https://padlet.com/mariefrancoisefronieux/coil-between-the-university-of-freiberg-tubaf-and-the-iut-un-xht6kjop91rmsxvs/wish/x5m7aoLAw5yBQkAV

Roomba should always maintain a distance of 15cm from the object. It adjust its position and view depending on where the object is located. This changes the perspective based on the movement and creates a simple, interactive connection between Roomba and the object.
![](https://avsuwzdjuo.cloudimg.io/v7/_cs_/2025/08/68b02be3ec562/image8.jpeg?w=412&org_if_sml=1)
---
## Code 
```C
#include <Arduino_Alvik.h>
#include <array>
#include <cmath>
#include <WiFi.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

const char* ssid = "roomba";
const char* password = "12345678";

WebsocketsServer wsServer;
WebsocketsClient client;

unsigned long lastSend = 0;
int counter = 0;

Arduino_Alvik alvik;

uint8_t* distance_map = nullptr;

float points[64][3];
float pos[2];
float rot;

int last_reconnect = millis();

void setup() {
  alvik.begin();

  Serial.begin(115200);

  delay(1000);

  WiFi.softAPConfig(IPAddress(10,0,0,1), IPAddress(10,0,0,1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);

  wsServer.listen(8080);
  Serial.println("WebSocket server started on port 8080");

  delay(5000);
}

void convert_tof_reading_to_obj_space() {
  for(int i = 0; i < 64; ++i) {
    float yaw = (30 - (60/8 * (i%8))) * (M_PI / 180.0);
    float pitch = (30 - (60/8 * (i/8))) * (M_PI / 180.0);

    float reading = distance_map[i] * 10.0; // convert to mm
    points[i][0] = reading;
    points[i][1] = reading * std::tan(yaw);
    points[i][2] = reading * std::tan(pitch);
  }
}

void convcert_obj_space_to_world_space() {
  float cos_rot = std::cos(rot * (M_PI / 180.0));
  float sin_rot = std::sin(rot * (M_PI / 180.0));

  for(int i = 0; i < 64; ++i) {
    float x_obj = points[i][0];
    float y_obj = points[i][1];

    float worldX = x_obj * cos_rot - y_obj * sin_rot + pos[0] * 10;
    float worldY = x_obj * sin_rot + y_obj * cos_rot + pos[1] * 10;

    points[i][0] = ((int)(worldX * 100.0))/100.0;
    points[i][1] = ((int)(worldY * 100.0))/100.0;
  }
}

int last_measure = 0;


const float FORWARD_SPEED = 12.0;   // cm/s
const float TURN_ANGLE    = 25.0;   // degrees
const int   SAFE_DISTANCE = 25;     // cm

float left, centerLeft, center, right, centerRight;

int scanDirection = 1;

void loop() {

  if(millis() - last_measure > 15000 || millis() < 15000) {
    Serial.println("Start scanning");
    delay(100);
    alvik.brake();
    if (!client.available()) {
      client = wsServer.accept();
      client.setFragmentsPolicy(FragmentsPolicy_Aggregate);
      if (client.available()) {
        Serial.println("Client connected");
      }
    }

    for(int i = 0; i < 36; ++i) {
        alvik.rotate(12 * scanDirection);
        alvik.request_distance_map(5, 5000);
        distance_map = alvik.get_distance_map();
        convert_tof_reading_to_obj_space();
        convcert_obj_space_to_world_space();
        alvik.get_pose(pos[0], pos[1], rot);

        if (client.available()) {
          Serial.println("Sending");
            client.sendBinary((char*)points, sizeof(points));
        }
        delay(100);
      }
    last_measure = millis();

    scanDirection *= -1;
  }

  // Read the five distance zones
  alvik.get_distance(left, centerLeft, center, centerRight, right);

  if (center < SAFE_DISTANCE ||
      centerLeft < SAFE_DISTANCE ||
      centerRight < SAFE_DISTANCE) {

    alvik.brake();
    delay(100);

    // Turn toward the side with more free space
    if ((left + centerLeft) > (right + centerRight)) {
      alvik.rotate(TURN_ANGLE);   // turn left
    } else {
      alvik.rotate(-TURN_ANGLE);    // turn right
    }

    delay(100);
  }
  else {
    // Path is clear
    alvik.drive(FORWARD_SPEED, 0.0);
  }

  delay(50);

}
```
---
## Mission 2: 3D scanner
The Roomba should digitally capture its surroundings through measurements and create a three-dimensional representation. The scanner collects distance data and process it to create a point cloud that shape and position of objects visible. The collected data is transmitted wirrelessy and displayed as a 3D model in real time.
![](https://images.interestingengineering.com/img/iea/QjOdvYq76d/1920px-3-proj2camsvg.png)
## Code example
Code block that runs on the STM, processes sensor data, and sends it to the ESP:
```C
    {
        {
          uint8_t reading_count = 0;
          packeter.unpacketC1B(code, reading_count);

          uint8_t readings[reading_count][64];
          uint8_t sigmas[reading_count][64];
          for(int i = 0; i < reading_count; i++){
            tof.update();
            for(int j = 0; j < 64; j++)
            {
              uint16_t d = tof.results.distance_mm[j];
              d = std::min<uint16_t>(d, static_cast<uint16_t>(2500));
              readings[i][j] = static_cast<uint8_t>(d / 10);

              uint16_t s = tof.results.range_sigma_mm[j];
              s = std::min<uint16_t>(s, static_cast<uint16_t>(2500));
              sigmas[i][j] = static_cast<uint8_t>(s / 10);
            }
            delay(100);
        }

          for(int i = 0; i < 64; i++)
          {
            uint8_t sigma;
            float num = 0;
            float denom = 0;
            for(int j = 0; j < reading_count; j++)
            {
              sigma = sigmas[j][i];
              num += readings[j][i] / ((float)sigma*sigma+ 1e-6);
              denom += 1.0 / ((float)sigma*sigma+ 1e-6);
            }

            readings[0][i] = num / denom;
          }

          msg_size = packeter.packetC64U('#', readings[0]);
          alvik.serial->write(packeter.msg,msg_size);
        }
    }

```
{{1}}
Counterpart on the ESP that requests the sensor data:
```C
void Arduino_Alvik::request_distance_map(const uint8_t readings_count, const uint16_t timeout)
{
  Serial.println("Requesting distance map");
  auto start_time = millis();
  distance_request_finished = false;
  while (!xSemaphoreTakeRecursive(buffer_semaphore, TICK_TIME_SEMAPHORE));
  msg_size = packeter->packetC1B('#', readings_count);
  uart->write(packeter->msg, msg_size);
  xSemaphoreGiveRecursive(buffer_semaphore);
  while(!distance_request_finished && (millis() - start_time < timeout))
  {
    delay(10);
  }
  if (!distance_request_finished)
  {
    Serial.println("Distance map request timed out");
  }
  else
  {
    Serial.println("Distance map request finished");
  }
}
```
  {{2}}
Code that requests sensor data, processes the values, and sends them to the computer:
```C
for(int i = 0; i < 36; ++i) 
    {
        alvik.rotate(12 * scanDirection);
        alvik.request_distance_map(5, 5000);
        distance_map = alvik.get_distance_map();
        convert_tof_reading_to_obj_space();
        convcert_obj_space_to_world_space();
        alvik.get_pose(pos[0], pos[1], rot);

        if (client.available()) 
        {
          Serial.println("Sending");
            client.sendBinary((char*)points, sizeof(points));
        }
        delay(100);
    }
```


---
## Challenges 

| Problems                                            | Solution                 |
| --------------------------------------------------- | ------------------------ |
| Existing libraries for the serial communication between the ESP and STM did not provide the required functionality.                 | The libraries were modified and adapted to the project requirements|  
| The Alvik library for the ESP and the carrier board did not fully support the needed functions.        | The library was adjusted and extended. |
| The large amount of sensor data (64 data points per measurement) caused slow data transmission.     | The sensor data was compressed before being sent to the ESP.       |
| The STM and ESP froze multiple times due to high processing load.               | The data processing and communication were optimized to reduce system overload.      |
| Communication between the computer and the Alvik still disconnects regularly.                 | This issue is still being investigated and requires further optimization.     |

---


## TUBAF and IUT
![](https://v1.padlet.pics/3/image.webp?t=c_limit%2Cdpr_2%2Ch_690%2Cw_1265&url=https%3A%2F%2Fu1.padletusercontent.com%2Fuploads%2Fpadlet-uploads-usc1%2F5562872267%2F686bc95ec4dec88362214d6e81988d9f%2FGemini_Generated_Image_rfr76mrfr76mrfr7.png%3Fexpiry_token%3D5WaHZRdGG3LkUVQGy3SZ-zdRtq89aJeottSBaF_Hii8dmxJqYDvE2-MDbblcM-ZrVekXW99RReKkJFIoMoKio1n1p81Mx7jp62XIDMEQdbyCGYBmvrNO1849fYhqEFqLEEBu5jIc8G-wIUoIaYV2-BtxKAqUX1aBphh4wyacwyWbLwYQdQBpUKypvpPofMpsLkrObJMwbZaCiR3IFg0uxQXSzw6z1SkkAzeKpZ7Lp2Wn6iEdG-YsvsCxYl3-vWzW)

- **<u>TUBAF:</u>** Moritz Bergander, Matteo Poma and Leonie Blum

- **<u>IUT:</u>** Maimouna Bathily and Moubarak Issa Lalo


**🕝 Time managment:** Meeting on Wednesday at 14:30 PM

**💬 Communication:** Through ouer WhatsApp group (by writing or voice calls)

**🎯 Goal:** Improve our English communication and skills


## IUT roboter

| Feature | 3D scanner robot | Line following robot|
|---------|------------------|---------------------
|**Task**| Captures the environment and creates a digital 3D representation| Moves autonomously along a black line|
|**Sensors**| TOF sensor for measuring distances and collecting measurement data| Light sensor for line detection and obstacle sensor for distance measurement|
|**Processing**|Processes large amounts of sensor data and creates a point cloud|Reacts to the detected line and obstacles by adjusting its movement|
|**Environmental response**|Updates the digital representation based on the measured environment|Stops or changes direction when obstacles are detected within 20 cm|
|**Display**|Visualizes the scanned environment as a 3D model in real time| Shows the current robot status using an LCD display|
---
https://www.capcut.com/view/7645081500274475536?workspaceId=7483069018971111477
## Reflection
**What we learned from this project:**
- Successful multicultural collaboration despite communication challenges with the French team members.
- Improve our English skills
  

**What we could do better next time:**
- better communication within the team
- better time managment


## Thank you for your attention
