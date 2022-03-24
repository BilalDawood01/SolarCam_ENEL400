// INCLUDE LIBRARIES
#include "Arduino.h"              // General Functionality
#include "esp_camera.h"           // Camera
#include <SD.h>                   // SD Card
#include "FS.h"                   // File System
#include "soc/soc.h"              // Disable brownout problems
#include "soc/rtc_cntl_reg.h"     // Disable brownout problems
#include "driver/rtc_io.h"
#include <EEPROM.h>               // read and write from flash memory
#include <WiFi.h>                 // WiFi Functionality
#include "time.h"                 // Time functions
#include "ESP_Mail_Client.h"      // e-Mail Functionality

#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void setup() 
{
  Serial.println("Ready... Set... Go!");
  // SETTING UP SERIAL COMMUNICATION -----------------------------------
  Serial.begin(115200);                         // Sets the data rate in bits per second (baud) for serial data transmission.
  
  // PIN DEFINITIONS FOR AI THINKER -----------------------------------
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;   //PIXFORMAT_GRAYSCALE

  Serial.println("Finished PIN Definitions.");
  
  // SELECTING IMAGE QUALITY BASED ON WHETHER "PSEUDO RAM" IS FOUND -----------------------------------
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;   // Highest Quality Selected; Alternatively: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;             // 0-63, lower means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  Serial.println("Selected Image Quality Based on Whether PSEUDO RAM was Found");

  // INITIALIZING THE CAMERA -----------------------------------
  delay(250);     // Delay to ensure interface is up and running before we try to turn the camera on
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
  } else {
    Serial.println("Initialized the Camera!");
  }
  
}

void loop() {}
