// INCLUDES
#include "Arduino.h" // General functionality
#include "esp_camera.h" // Camera
#include <SD.h> // SD Card 
#include "SD_MMC.h"
#include "FS.h" // File System
#include "soc/soc.h" // System settings (e.g. brownout)
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <EEPROM.h> // EEPROM flash memory
#include <WiFi.h> // WiFi
#include "time.h" // Time functions
// "ESP Mail Client" by Mobizt, tested with v1.6.4
#include "ESP_Mail_Client.h" // e-Mail

// DEFINES
#define FPS 1                                // Frames Per Second
#define LoopDelay 1000/FPS                    // Delay Between Frames

// Wi-Fi settings
#define WIFI_SSID "network"
#define WIFI_PASSWORD "password"

/* ----- ----- SET MOTION SENSOR INPUT PIN AND LEDPin OUTPUT PIN VALUES ----- ----- */          
gpio_num_t MSPin = GPIO_NUM_13;
String path;


/* ----- ----- MAIN PROGRAM ----- ----- */

void init_cam(){
    // Pin definition for CAMERA_MODEL_AI_THINKER
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
  config.pixel_format = PIXFORMAT_JPEG;

  // If the board has additional "pseudo RAM", we can create larger images
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // UXGA=1600x1200. Alternative values: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

    // Initialise the camera
  // Short pause helps to ensure the I2C interface has initialised properly before attempting to detect the camera
  delay(250);
  esp_err_t err = esp_camera_init(&config);
  while (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    err = esp_camera_init(&config);
  }
}

void take_pic(){
    // We want to take the picture as soon as possible after the sensor has been triggered, so we'll do that first, before
  // setting up the SD card, Wifi etc.
  // Initialise a framebuffer 
  camera_fb_t *fb = NULL;
  // But... we still need to give the camera a few seconds to adjust the auto-exposure before taking the picture
  // Otherwise you get a green-tinged image as per https://github.com/espressif/esp32-camera/issues/55
  // Two seconds should be enough
  delay(250);
  // Take picture
  fb = esp_camera_fb_get();
  // Check it was captured ok  
  if(!fb) {
    Serial.println("Camera capture failed");
    fb = esp_camera_fb_get();
  }

  // Turn flash off after taking picture
//  ledcWrite(7, 0);

  // Build up the string of the filename we'll use to save the file
  path = "/pic";
    
    // Following section creates filename based on timestamp
    const long gmtOffset_sec = -6;
    const int daylightOffset_sec = 0;
    // Synchronise time from specified NTP server - e.g. "pool.ntp.org", "time.windows.com", "time.nist.gov"
    // From https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Time/SimpleTime/SimpleTime.ino
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");
    struct tm timeinfo;
    while(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
    }
    Serial.print("Current time is ");
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    char timeStringBuff[50]; //50 chars should be enough
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y%m%d_%H%M%S", &timeinfo);
    path += (String)timeStringBuff;

  // Add the file extension
  path += ".jpg";

  // Next, we need to start the SD card
  Serial.println("Starting SD Card");
  while(!MailClient.sdBegin(14, 2, 15, 13)) {
    Serial.println("SD Card Mount Failed");
  }
  
  // Access the file system on the SD card
  fs::FS &fs = SD;
  // Attempt to save the image to the specified path
  File file = fs.open(path.c_str(), FILE_WRITE);
  while(!file){
    Serial.printf("Failed to save to path: %s\n", path.c_str());
    file = fs.open(path.c_str(), FILE_WRITE);
  }
  
  file.write(fb->buf, fb->len); // payload (image), payload length
  Serial.printf("Saved file to path: %s\n", path.c_str());
    
  file.close();

  // Now that we've written the file to SD card, we can release the framebuffer memory of the camera
  esp_camera_fb_return(fb);

}

void setup() 
{
  Serial.println("Woke up");
  // SETTING UP SERIAL COMMUNICATION -----------------------------------
  Serial.begin(115200);                         // Sets the data rate in bits per second (baud) for serial data transmission.
  // CAUTION - We'll disable the brownout detection
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  init_cam();
  
  // SETTING UP PINS -----------------------------------
  pinMode(MSPin, INPUT_PULLUP);                 // Set PIR Motion Sensor mode to INPUT_PULLUP

  
  // Connect to Wi-Fi if required
  
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("Big_Bro_Inc_Security_Cam");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }
    if(WiFi.isConnected()){
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print(" Signal Level: ");
      Serial.println(WiFi.RSSI());
      Serial.println();
    }
    else {
      Serial.println("Failed to connect to Wi-Fi");
      
    }
  
}

void loop() {
  pinMode(MSPin, INPUT_PULLUP);                 // Set PIR Motion Sensor mode to INPUT_PULLUP
  while(digitalRead(MSPin)){
      Serial.println("DigitalRead High"); 
      gpio_reset_pin(MSPin);
      take_pic();
      delay(LoopDelay);
  }

  Serial.println("DigitalRead Low");
  delay(LoopDelay);
}
