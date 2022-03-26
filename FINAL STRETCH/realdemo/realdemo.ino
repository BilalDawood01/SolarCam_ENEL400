// INCLUDE LIBRARIES
//#include "Arduino.h"              // General Functionality
//#include "esp_camera.h"
//#include "SD_MMC.h"                   // SD Card
//#include "FS.h"                   // File System
//#include "soc/soc.h"              // Disable brownout problems
//#include "soc/rtc_cntl_reg.h"     // Disable brownout problems
//#include "driver/rtc_io.h" 
//#include "time.h"                 // Time functions
//#include <WiFi.h>

//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#include "camera_pins.h"

// Define minimal time to record after every sensor activation 
#define waitTime 3                            // Wait Time defined in seconds
#define FPS 1                                 // Frames Per Second
#define LoopDelay 1000/FPS                    // Delay Between Frames


/* ----- ----- SET MOTION SENSOR INPUT PIN AND LEDPin OUTPUT PIN VALUES ----- ----- */
const int MSPin = 14;

/* ----- ----- MAIN PROGRAM ----- ----- */
void setup() 
{
  Serial.println("Woke up");
  // SETTING UP SERIAL COMMUNICATION -----------------------------------
  Serial.begin(115200);                         // Sets the data rate in bits per second (baud) for serial data transmission.

  // SETTING UP PINS -----------------------------------
  pinMode(MSPin, INPUT_PULLUP);                 // Set PIR Motion Sensor mode to INPUT_PULLUP

  unsigned long lastTrigger = 0;
  while(digitalRead(MSPin) || millis() - lastTrigger < waitTime*1000){
    if(digitalRead(MSPin)){
      lastTrigger = millis();
      Serial.println("DigitalRead High"); 
    } else {
      Serial.println("DigitalRead Low but Within Wait Time"); 
    }
    
    delay(LoopDelay);
  }

  Serial.println("DigitalRead Low");

  // GOING TO DEEP SLEEP -----------------------------------
  Serial.println("Going to Sleep...");
  Serial.flush();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,1);
  esp_deep_sleep_start();
}

void loop() {}
