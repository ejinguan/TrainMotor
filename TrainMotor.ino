// Based on https://github.com/KensCode/TrainMotor/

#include <TrainControl.h>
#include <Wire.h>
#include <CommandQueue.h>


#define useSpeedCurve false





/* Train Controller *************************************/

TrainControl myTC_A;
TrainControl myTC_B;


/* Pins for Motor Driver ********************************/

#define _pin_enA 9
#define _pin_inA1 8
#define _pin_inA2 11

#define _pin_enB 10
#define _pin_inB1 12
#define _pin_inB2 13


/* Direction Selection & LED ****************************/

#define _pin_dirA 2
#define _pin_dirB 3

#define _pin_LEDdirA1 4
#define _pin_LEDdirA2 5
#define _pin_LEDdirB1 6
#define _pin_LEDdirB2 7

bool lastPress_A = false;
bool lastPress_B = false;

int desiredDir_A = 0;
int desiredDir_B = 0;
int dir_A = 0;
int dir_B = 0;


/* Pins for Direction Selection *************************/

#define _eStop 6

#define TM_FAST_PWM 0

#define MIN_SPEED_STOPPED 30
#define MIN_SPEED_RUNNING 20

// Steps to consume 1 accel/decel step
#define ACCEL_PER_SEC 50
#define DECEL_PER_SEC 500


/* I2C Command Processing *******************************/

#define I2C_SpeedController 11
#define I2C_PointsController 12

CommandQueue queue(10);
String strI2CCommand;         // for all incoming command data
String strCommand;


/* For Serial Communication *****************************/

long loopcounter = 0;


/* Points Tracking **************************************/

// For tracking status of motor A and B - to determine if tracks can be switched
// Communication to other Arduinos
bool bIsRunningA = false;
bool bIsRunningB = false;
bool bIsLastRunningA = false;
bool bIsLastRunningB = false;

bool bP1State = false;
bool bP2State = false;
bool bIsInterlocked = false;



// From: https://docs.arduino.cc/learn/communication/wire
// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  char c;
  
  while(Wire.available() > 0) {         // loop through all characters
    c = Wire.read();                    // receive byte as a character
    if (c == '\n') {                    // if newline received, add to queue
      if (strI2CCommand.length() > 0) { // Add to queue if length is > 0
        queue.push(strI2CCommand);
        strI2CCommand = "";
      }
    } else {
      strI2CCommand += c;               // append character
    }
    Serial.print(c);                    // print the character
  }
  Serial.println('\n');                 // print new line
  
  if (strI2CCommand.length() > 0) {     // Add to queue if length is > 0
    queue.push(strI2CCommand);
    strI2CCommand = "";                 // Reset to blank
  }
}



void setup() {

  // Set direction button inputs as INPUT PULLUP  
  pinMode(_pin_dirA, INPUT_PULLUP);
  pinMode(_pin_dirB, INPUT_PULLUP);


  // Change prescale of Timer1 PWM ========================================
  
  // 16 MHz / 1 / 256 = 62500 kHz
  //int prescale = _BV(CS10); //_BV(CS11);
  
  // 16 MHz / 256 / 256 = 244 Hz CS12
  // 16 MHz / 1024 / 256 = 61 Hz CS12 + CS10
  int prescale = _BV(CS12); //  _BV(CS11);
  
  // WGM12 + WGM10 = 0101 -> Fast PWM to 255
  //         WGM10 = 0001 -> Phase-correct PWM to 255
  // Set Motor A (pin 9) to use Timer1A and Motor B (pin 10) to use Timer1B
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  if (TM_FAST_PWM == 1) TCCR1B = _BV(WGM12) | prescale; // Fast PWM
  else                  TCCR1B =              prescale; // Phase-correct PWM

  
  // Set up Train Controller objects ======================================

  myTC_A.setPins(_pin_inA1, _pin_inA2, _pin_enA);
  myTC_A.setAcceleration(ACCEL_PER_SEC, DECEL_PER_SEC);
  myTC_A.setMinSpeed(MIN_SPEED_STOPPED, MIN_SPEED_RUNNING);
  
  myTC_B.setPins(_pin_inB1, _pin_inB2, _pin_enB);
  myTC_B.setAcceleration(ACCEL_PER_SEC, DECEL_PER_SEC);
  myTC_B.setMinSpeed(MIN_SPEED_STOPPED, MIN_SPEED_RUNNING);

  
  // Start communications
  
  Serial.begin(115200);
  Serial.print  ("raw_A,SP_A,PV_A,PV_final_A,dir_A,");
  Serial.println("raw_B,SP_B,PV_B,PV_final_B,dir_B");

  Wire.begin(I2C_SpeedController);                            // Join as slave
  bitSet(TWAR, TWGCE);                                        // Enable I2C General Call receive (broadcast) in TWI Addr Register
  Wire.onReceive(receiveEvent);                               // Register event handler for I2C
}

void loop() {
  
  /**********************************************************************************
  /* Step 1: Process incoming commands on Serial and I2C
  /**********************************************************************************/

  // If there is a new command
  if (queue.size() > 0) {
    // Get the first command
    strCommand = queue.pop();

    if (strCommand=="_p1d") {
      bP1State = true;
    } else if (strCommand=="_p1t") {
      bP1State = false;
    } else if (strCommand=="_p2d") {
      bP2State = true;
    } else if (strCommand=="_p2t") {
      bP2State = false;
    }
    
  } else {
    strCommand = "";
  }
  

  /**********************************************************************************
  /* Step X: Read and process direction status
  /**********************************************************************************/
  
  // If pin is not the same as lastPress, delay some time for debounce
  if (digitalRead(_pin_dirA)!=lastPress_A) myTC_A.activeDelayMillis(50);
  desiredDir_A = digitalRead(_pin_dirA)==LOW?TM_EAST:TM_WEST;
  
  // If pin is not the same as lastPress, delay some time for debounce
  if (digitalRead(_pin_dirB)!=lastPress_B) myTC_B.activeDelayMillis(50);
  desiredDir_B = digitalRead(_pin_dirB)==LOW?TM_EAST:TM_WEST;  

  // Set direction if it's stopped
  if (myTC_A.getSpeedFinal()==0) myTC_A.setDirection(desiredDir_A);
  if (myTC_B.getSpeedFinal()==0) myTC_B.setDirection(desiredDir_B);
  
  // Keep updating the current pin status
  lastPress_A = digitalRead(_pin_dirA);
  lastPress_B = digitalRead(_pin_dirB);



  /**********************************************************************************
  /* Step X: Read and process speed control
  /**********************************************************************************/
  
  // Read speed control input
  int speedIn_A = 0;
  int speedIn_B = 0;
  
  // Check track status
  if (bP1State || bP2State) { // either track is already diverted
    bIsInterlocked = true;
    
    if (!bIsRunningB) { // If B is not already running, read A speed
      speedIn_A = analogRead(A0);
    }
    if (!bIsRunningA) { // If A is not already running, read B speed
      speedIn_B = analogRead(A1);
    }  
  } else { // Tracks are not diverted. Read both signals
    bIsInterlocked = false;
    
    speedIn_A = analogRead(A0);
    speedIn_B = analogRead(A1);
  }
  
  //int rawMotorSpeed = map(xAxis, 50, 900, 0, 255); // Map 50-900 to 0-255
  int rawMotorSpeed_A = SpeedCurve(speedIn_A);
  int rawMotorSpeed_B = SpeedCurve(speedIn_B);
  
  myTC_A.setSpeed(rawMotorSpeed_A);
  myTC_B.setSpeed(rawMotorSpeed_B);

  myTC_A.updateThrottles(); 
  myTC_B.updateThrottles();
  
  bIsRunningA = (myTC_A.getSpeedFinal() > 0);
  bIsRunningB = (myTC_B.getSpeedFinal() > 0);



  /**********************************************************************************
  /* Step X: Update Serial, LED, I2C
  /**********************************************************************************/

  // Broadcast on I2C  
  if (bIsRunningA != bIsLastRunningA) {
    Wire.beginTransmission(I2C_PointsController);
    Wire.write(bIsRunningA?"s1on":"s1off"); // Sends s1on for run, s1off for stopped
    Wire.endTransmission();    
  }
  if (bIsRunningB != bIsLastRunningB) {
    Wire.beginTransmission(I2C_PointsController);
    Wire.write(bIsRunningB?"s2on":"s2off"); // Sends s2on for run, s2off for stopped
    Wire.endTransmission();    
  }


  UpdateLEDs();


  /******** Update on Serial output if necessary ********/
  if (loopcounter % 50 == 0) {
        
    // Motor A
    Serial.print(rawMotorSpeed_A);
    Serial.print(",");
    
    Serial.print(myTC_A.getSpeedSetpoint());
    Serial.print(",");
    
    Serial.print(myTC_A.getSpeed());
    Serial.print(",");
    
    Serial.print(myTC_A.getSpeedFinal());
    Serial.print(",");

    Serial.print(",");
    
    // Motor B
    Serial.print(rawMotorSpeed_B);
    Serial.print(",");
    
    Serial.print(myTC_B.getSpeedSetpoint());
    Serial.print(",");
    
    Serial.print(myTC_B.getSpeed());
    Serial.print(",");
    
    Serial.print(myTC_B.getSpeedFinal());
    Serial.print(",");
    
  }

  loopcounter++;
  
  // delay 1ms for debounce
  //myTC_A.activeDelayMillis(1);
}

int SpeedCurve(int inSpeed) {
  if(useSpeedCurve) {
    if (inSpeed <= 200) return map (inSpeed,  50, 200,   0,  20);
    if (inSpeed <= 400) return map (inSpeed, 200, 400,  20,  50);
    if (inSpeed <= 600) return map (inSpeed, 400, 600,  50, 100);
    if (inSpeed <= 800) return map (inSpeed, 600, 900, 100, 175);
    /*else*/            return map (inSpeed, 800, 900, 175, 255);
  } else {
    return map(inSpeed, 50, 900, 0, 255);
  }
}

void UpdateLEDs() {
  bool bLightsOff = false;
  bool bFlashA = false;
  bool bFlashB = false;

  // Turn off the LED that is off if current seconds is odd
  bLightsOff = (((millis() / 1000) % 2) == 1);

  bFlashA = (bIsInterlocked && bIsRunningB);
  bFlashB = (bIsInterlocked && bIsRunningA);
  
  // Update Direction LEDs  
  dir_A = myTC_A.getDirection();
  Serial.print(dir_A);
  digitalWrite(_pin_LEDdirA1, (dir_A==TM_EAST && !bFlashA)?HIGH:LOW);
  digitalWrite(_pin_LEDdirA2, (dir_A==TM_WEST && !bFlashA)?HIGH:LOW);

  dir_B = myTC_B.getDirection();
  Serial.println(dir_B);  
  digitalWrite(_pin_LEDdirB1, (dir_B==TM_EAST && !bFlashB)?HIGH:LOW);
  digitalWrite(_pin_LEDdirB2, (dir_B==TM_WEST && !bFlashB)?HIGH:LOW);
  
}
