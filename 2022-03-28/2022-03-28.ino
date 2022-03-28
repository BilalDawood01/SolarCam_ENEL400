// DEFINES
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
  
}

void loop() {
  while(digitalRead(MSPin)){
      Serial.println("DigitalRead High"); 
      delay(LoopDelay);
  }

  Serial.println("DigitalRead Low");
  delay(LoopDelay);
}
