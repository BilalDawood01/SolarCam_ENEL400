// INCLUDE STATEMENTS
#include "Arduino.h"              // General Functionality
#include "esp_camera.h"           // Camera
#include "SD_MMC.h"
#include "FS.h"                   // File System
#include "SPI.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "ESP_Mail_Client.h"      // e-Mail Functionality

// CAMERA_MODEL_AI_THINKER
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

// Define minimal time to record after every sensor activation 
#define waitTime 2                            // Wait Time defined in seconds
#define FPS 1                                // Frames Per Second
#define LoopDelay 1000/FPS                    // Delay Between Frames

/* ----- ----- SET MOTION SENSOR INPUT PIN AND LEDPin OUTPUT PIN VALUES ----- ----- */             
const int MSPin = 13;

/* ----- ----- MAIN PROGRAM ----- ----- */
void setup() 
{
  Serial.println("Woke up");
  // SETTING UP SERIAL COMMUNICATION -----------------------------------
  Serial.begin(115200);                         // Sets the data rate in bits per second (baud) for serial data transmission.

  // SETTING UP PINS -----------------------------------
  pinMode(MSPin, INPUT_PULLUP);                 // Set PIR Motion Sensor mode to INPUT_PULLUP

  init_SD_card_and_cam();
  
  unsigned long lastTrigger = 0;
  while(digitalRead(MSPin)){
   Serial.println("DigitalRead High"); 
   delay(LoopDelay);
  }

  Serial.println("DigitalRead Low");

  sleep();
}

void loop() {}

void init_SD_card_and_cam() {
  // Specify that LED pin will be low
    esp_err_t ret = ESP_FAIL;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // this tells SD host to keep using 1-line bus mode
    host.flags = SDMMC_HOST_FLAG_1BIT;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    // this tells the driver to only initialize 1 data line
    slot_config.width = 1;
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 10,
    };
    sdmmc_card_t *card;
  
    Serial.println("Mounting SD card...");
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
  
    if (ret == ESP_OK) {
      Serial.println("SD card mount successfully!");
    }  else  {
      Serial.printf("Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
    }

    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
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
}

void sleep() {
  Serial.println("Going to Sleep...");
  Serial.flush();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,1);
  esp_deep_sleep_start();
}
