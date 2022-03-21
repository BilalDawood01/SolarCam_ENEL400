
// Define minimal time to record after every sensor activation 
#define waitTime 5                            // Wait Time defined in seconds
#define FPS 1                                // Frames Per Second
#define LoopDelay 1000/FPS                    // Delay Between Frames

/* ----- ----- SET MOTION SENSOR INPUT PIN AND LEDPin OUTPUT PIN VALUES ----- ----- */
const int LEDPin = 13;               
const int MSPin = 14;

/* ----- ----- MAIN PROGRAM ----- ----- */
void setup() 
{
  Serial.println("Woke up");
  // SETTING UP SERIAL COMMUNICATION -----------------------------------
  Serial.begin(115200);                         // Sets the data rate in bits per second (baud) for serial data transmission.

  // SETTING UP PINS -----------------------------------
  pinMode(MSPin, INPUT_PULLUP);                 // Set PIR Motion Sensor mode to INPUT_PULLUP
  pinMode(LEDPin, OUTPUT);                      // Set LEDPin pin as an output
  digitalWrite(LEDPin, LOW);                    // Set LEDPin pin to LOW

  unsigned long lastTrigger = 0;
  while(digitalRead(MSPin) || millis() - lastTrigger < waitTime*1000){
    if(digitalRead(MSPin)){
      lastTrigger = millis();
      Serial.println("DigitalRead High"); 
    } else {
      Serial.println("DigitalRead Low but Within Wait Time"); 
    }
    
    digitalWrite(LEDPin, HIGH);                 // Set LEDPin pin to HIGH
    delay(LoopDelay);
  }

  Serial.println("DigitalRead Low");
  digitalWrite(LEDPin, LOW);                    // Set LEDPin pin to LOW

  // GOING TO DEEP SLEEP -----------------------------------
  Serial.println("Going to Sleep...");
  Serial.flush();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,1);
  esp_deep_sleep_start();
}

void loop() {}
