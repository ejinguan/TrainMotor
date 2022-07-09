// Based on https://github.com/KensCode/TrainMotor/

#include <TrainControl.h>

#define useSpeedCurve false

#define _enA 9
#define _inA1 8
#define _inA2 11

#define _enB 10
#define _inB1 12
#define _inB2 13

#define _dirA 2
#define _dirB 3

#define _LEDdirA1 4
#define _LEDdirA2 5
#define _LEDdirB1 6
#define _LEDdirB2 7

#define _eStop 6

#define TM_FAST_PWM 0

#define MIN_SPEED_STOPPED 30
#define MIN_SPEED_RUNNING 20

// Steps to consume 1 accel/decel step
#define ACCEL_PER_SEC 50
#define DECEL_PER_SEC 500

bool lastPress_A = false;
bool lastPress_B = false;
long loopcounter = 0;

int desiredDir_A = 0;
int desiredDir_B = 0;
int dir_A = 0;
int dir_B = 0;

TrainControl myTC_A;
TrainControl myTC_B;

void setup() {
  
  pinMode(_dirA, INPUT_PULLUP);
  pinMode(_dirB, INPUT_PULLUP);
  
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


  myTC_A.setPins(_inA1, _inA2, _enA);
  myTC_A.setAcceleration(ACCEL_PER_SEC, DECEL_PER_SEC);
  myTC_A.setMinSpeed(MIN_SPEED_STOPPED, MIN_SPEED_RUNNING);
  
  myTC_B.setPins(_inB1, _inB2, _enB);
  myTC_B.setAcceleration(ACCEL_PER_SEC, DECEL_PER_SEC);
  myTC_B.setMinSpeed(MIN_SPEED_STOPPED, MIN_SPEED_RUNNING);
  
  Serial.begin(115200);
  Serial.print  ("raw_A,SP_A,PV_A,PV_final_A,dir_A,");
  Serial.println("raw_B,SP_B,PV_B,PV_final_B,dir_B");
}

void loop() {

  // If pin is not the same as lastPress, delay some time for debounce
  if (digitalRead(_dirA)!=lastPress_A) myTC_A.activeDelayMillis(50);
  desiredDir_A = digitalRead(_dirA)==LOW?TM_EAST:TM_WEST;
  
  // If pin is not the same as lastPress, delay some time for debounce
  if (digitalRead(_dirB)!=lastPress_B) myTC_B.activeDelayMillis(50);
  desiredDir_B = digitalRead(_dirB)==LOW?TM_EAST:TM_WEST;  

  // Set direction if it's stopped
  if (myTC_A.getSpeedFinal()==0) myTC_A.setDirection(desiredDir_A);
  if (myTC_B.getSpeedFinal()==0) myTC_B.setDirection(desiredDir_B);
  
  // Keep updating the current pin status
  lastPress_A = digitalRead(_dirA);
  lastPress_B = digitalRead(_dirB);

  // Read speed control input
  int speedIn_A = analogRead(A0);
  int speedIn_B = analogRead(A1);
  //int rawMotorSpeed = map(xAxis, 50, 900, 0, 255); // Map 50-900 to 0-255
  int rawMotorSpeed_A = SpeedCurve(speedIn_A);
  int rawMotorSpeed_B = SpeedCurve(speedIn_B);
  
  myTC_A.setSpeed(rawMotorSpeed_A);
  myTC_B.setSpeed(rawMotorSpeed_B);

  myTC_A.updateThrottles(); 
  myTC_B.updateThrottles(); 
  
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

    dir_A = myTC_A.getDirection();
    Serial.print(dir_A);
    digitalWrite(_LEDdirA1, (dir_A==TM_EAST)?HIGH:LOW);
    digitalWrite(_LEDdirA2, (dir_A==TM_WEST)?HIGH:LOW);

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

    dir_B = myTC_B.getDirection();
    Serial.println(dir_B);
    digitalWrite(_LEDdirB1, (dir_B==TM_EAST)?HIGH:LOW);
    digitalWrite(_LEDdirB2, (dir_B==TM_WEST)?HIGH:LOW);
    
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
