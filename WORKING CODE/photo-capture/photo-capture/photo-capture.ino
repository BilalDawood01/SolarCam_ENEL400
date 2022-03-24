// INCLUDE LIBRARIES
#include "esp_log.h"
#include "esp_http_server.h"
#include "Arduino.h"              // General Functionality
#include "esp_camera.h"           // Camera
#include <SD.h>                   // SD Card
#include "FS.h"                   // File System
#include "soc/soc.h"              // Disable brownout problems
#include "soc/rtc_cntl_reg.h"     // Disable brownout problems
#include "driver/rtc_io.h"
//#include <EEPROM.h>               // read and write from flash memory
//#include <WiFi.h>                 // WiFi Functionality
#include "time.h"                 // Time functions
#include "ESP_Mail_Client.h"      // e-Mail Functionality

// Define minimal time to record after every sensor activation 
#define waitTime 5                            // Wait Time defined in seconds
#define FPS 1                                // Frames Per Second
#define LoopDelay 1000/FPS                    // Delay Between Frames

/* ----- ----- SET MOTION SENSOR INPUT PIN ----- ----- */            
const int MSPin = 14;
uint16_t picNum = 1;

/* ----- ----- OFT-REPEATED SLEEP FUNCTION ----- ----- */
void sleep(){
  Serial.println("Going to Sleep...");
  Serial.flush();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,1);
  esp_deep_sleep_start();
}

/* ----- ----- MAIN PROGRAM ----- ----- */
void setup() 
{
  Serial.println("Woke up");
  // SETTING UP SERIAL COMMUNICATION -----------------------------------
  Serial.begin(115200);                         // Sets the data rate in bits per second (baud) for serial data transmission.

  // SETTING UP PINS -----------------------------------
  pinMode(MSPin, INPUT_PULLUP);                 // Set PIR Motion Sensor mode to INPUT_PULLUP

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

  // INITIALIZING THE CAMERA -----------------------------------
  delay(250);     // Delay to ensure interface is up and running before we try to turn the camera on
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    sleep();
  }
  
  // TAKING PHOTO BASED ON INPUT PIN STATE -----------------------------------
  unsigned long lastTrigger = 0;
  while(digitalRead(MSPin) || millis() - lastTrigger < waitTime*1000){
    if(digitalRead(MSPin)){
      lastTrigger = millis();
      Serial.println("DigitalRead High"); 
    } else {
      Serial.println("DigitalRead Low but Within Wait Time"); 
    }

    // Initialize memory for frame buffer
      camera_fb_t * fb = NULL;
      fb = esp_camera_fb_get();       // Take Picture!
      if (!fb) {
        Serial.println("Camera capture failed");
        sleep();
      }

    // Create Path
    String path = "/pic - " + String(picNum++) + ".jpg";

    // Writing to SD Card
    Serial.println("Starting SD Card...");
    if(!MailClient.sdBegin(14,2,15,13)){
      Serial.println("SD Card Mount Failed");
      sleep();
    }

    fs::FS &fs = SD;
    File file = fs.open(path.c_str(), FILE_WRITE);
    if(!file){
      Serial.printf("Failed to save to path: %s\n", path.c_str());
      sleep();
    } else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.printf("Saved file to path: %s\n", path.c_str());
    }
    file.close();
    esp_camera_fb_return(fb);
    
    delay(LoopDelay);
  }

  Serial.println("DigitalRead Low");

  // GOING TO DEEP SLEEP -----------------------------------
  sleep();
}

void loop() {}
