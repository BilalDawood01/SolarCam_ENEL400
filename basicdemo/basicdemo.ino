
// Define minimal time to record after every sensor activation 
#define timeSeconds 15

/* ----- ----- SET MOTION SENSOR INPUT PIN AND LED OUTPUT PIN VALUES ----- ----- */
const int led = 13;               
const int motionSensor = 14;

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long then = 0;
unsigned long lastTrigger = 0;
boolean startTimer = false;


/* ----- ----- INTERRUPT SERVICE ROUTINE TRIGGERED BY PIR SENSOR ----- ----- */
void IRAM_ATTR detectsMovement() 
{
  if(startTimer == false){
    Serial.println("MOTION DETECTED!!!");
    // print message to serial monitor
    
    digitalWrite(led, HIGH);
    // Set LED pin to HIGH
    
    startTimer = true;
    
    lastTrigger = millis();
  }
}


/* ----- ----- INITIALIZATION ----- ----- */
void setup() 
{
  Serial.begin(115200);          
  // Sets the data rate in bits per second (baud) for serial data transmission.

  // SETTING UP PINS
  pinMode(motionSensor, INPUT_PULLUP);
  // Set PIR Motion Sensor mode to INPUT_PULLUP
  pinMode(led, OUTPUT);
  // Set LED pin as an output
  digitalWrite(led, LOW);
  // Set LED pin to LOW

  // MOTION SENSOR INTERRUPT
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);
  // Set motionSensor pin as interrupt, assign interrupt function "detectsMovement" to run when activated, and set RISING mode
}

/* ----- ----- MAIN PROGRAM - CONTINUOUS LOOP ----- ----- */
void loop() 
{
  
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  now = millis();
  if(startTimer){
      if((now - lastTrigger > (timeSeconds*1000))){
      if(digitalRead(motionSensor)){
        lastTrigger = millis();
      } else {
        Serial.println("Motion stopped...");
        digitalWrite(led, LOW);
        startTimer = false;
      }
    }
  }

//  // FOR TESTING PURPOSES: GIVE MESSAGE EVERY 2 SECONDS INDICATED ESP32 IS ON
//  if (now > then + 2000)
//  {
//    then = now;
//    Serial.println("It is working...");
//  }
  
}
