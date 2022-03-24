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
#include "ESP_Mail_Client.h"      // e-Mail Functionality
#include <ESPmDNS.h>
#include "ESP32FtpServer.h"
#include <HTTPClient.h>

FtpServer ftpSrv;   //set #define FTP_DEBUG in ESP32FtpServer.h to see ftp verbose on serial

// Time
#include "time.h"                 // Time functions

#include "lwip/err.h"
#include "lwip/apps/sntp.h"

// MicroSD
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

// SETTINGS! 
// Define minimal time to record after every sensor activation 
#define waitTime 5                            // Wait Time defined in seconds
#define FPS 1                                // Frames Per Second
#define LoopDelay 1000/FPS                    // Delay Between Frames

/* ----- ----- SET MOTION SENSOR INPUT PIN ----- ----- */            
const int MSPin = 14;
uint16_t picNum = 1;

long current_millis;
long last_capture_millis = 0;
static esp_err_t cam_err;
static esp_err_t card_err;
char strftime_buf[64];
int file_number = 0;
bool internet_connected = false;
struct tm timeinfo;
time_t now;

char *filename ;
char *stream ;
int newfile = 0;
int frames_so_far = 0;
FILE *myfile;
long bp;
long ap;
long bw;
long aw;
long totalp;
long totalw;
float avgp;
float avgw;
int overtime_count = 0;

// GLOBALS
#define BUFFSIZE 512

// global variable used by these pieces

char str[20];
uint16_t n;
uint8_t buf[BUFFSIZE];

static int i = 0;
uint8_t temp = 0, temp_last = 0;
unsigned long fileposition = 0;
uint16_t frame_cnt = 0;
uint16_t remnant = 0;
uint32_t length = 0;
uint32_t startms;
uint32_t elapsedms;
uint32_t uVideoLen = 0;
bool is_header = false;
long bigdelta = 0;
int other_cpu_active = 0;
int skipping = 0;
int skipped = 0;

int fb_max = 12;

camera_fb_t * fb_q[30];
int fb_in = 0;
int fb_out = 0;

camera_fb_t * fb = NULL;

FILE *avifile = NULL;
FILE *idxfile = NULL;

//
//
// EDIT ssid and password
//
// zzz
const char* ssid = "Network";
const char* password = "Password";

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
