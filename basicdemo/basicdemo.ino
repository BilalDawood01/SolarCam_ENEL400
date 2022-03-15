
// Define minimal time to record after every sensor activation 
#define timeSeconds 5

/* ----- ----- SET MOTION SENSOR INPUT PIN AND LEDPin OUTPUT PIN VALUES ----- ----- */
const int LEDPin = 13;               
const int MSPin = 14;

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long then = 0;
unsigned long lastTrigger = 0;
boolean startTimer = false;


/* ----- ----- INTERRUPT SERVICE ROUTINE TRIGGERED BY PIR SENSOR ----- ----- */
//void IRAM_ATTR detectsMovement() 
//{
//    Serial.println("ISR Ran!");
//    if(!digitalRead(MSPin) && startTimer == false){
//      Serial.println("1st");
//      lastTrigger = millis();
//      
//      digitalWrite(LEDPin, HIGH);
//      // Set LEDPin pin to HIGH
//      
//      startTimer = true;
//    } else if (digitalRead(MSPin) && startTimer == true){
//      Serial.println("2nd");
//      digitalWrite(LEDPin, LOW);
//      startTimer = false;
//    } else {
//      Serial.println("3rd");
//    }
//}


/* ----- ----- INITIALIZATION ----- ----- */
void setup() 
{
  Serial.println("Woke up");
  // SETTING UP SERIAL COMMUNICATION
  Serial.begin(115200);          
  // Sets the data rate in bits per second (baud) for serial data transmission.

  // SETTING UP PINS
  pinMode(MSPin, INPUT_PULLUP);
  // Set PIR Motion Sensor mode to INPUT_PULLUP
  pinMode(LEDPin, OUTPUT);
  // Set LEDPin pin as an output
  digitalWrite(LEDPin, LOW);
  // Set LEDPin pin to LOW

    while(digitalRead(MSPin)){
      Serial.println("DigitalRead High"); 
      delay(2000);
    }
  
    Serial.println("DigitalRead Low");
  // MOTION SENSOR INTERRUPT
//  attachInterrupt(digitalPinToInterrupt(MSPin), detectsMovement, CHANGE);
  // Set MSPin pin as interrupt, assign interrupt function "detectsMovement" to run when activated, and set RISING mode
    Serial.println("Going to Sleep...");
    delay(1000);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,1);
    esp_deep_sleep_start();
}

/* ----- ----- MAIN PROGRAM - CONTINUOUS LOOP ----- ----- */
void loop() 
{
 

}
