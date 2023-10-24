#include <MeRGBLineFollower.h>
#include <MePort.h>
#include <MeMegaPi.h>

/*
 * Main fileº
 * 
 * apply a duty cycle of 255/ref to the motor1 (PWM) where ref is a value obtained from the serial port (in ascii format) 
 * send back the actual position of the motor shaft (in pulses).
 * 
 */


#include "Motors.h"
#include "Encoders.h"
#include "lineFollower.h"
#include "Ultrasonic.h"


#define DT 50 //sampling period in milliseconds
#define PWMT 70 //définit le pwm de travail, celui d'un robot parfaitement calibré
#define KP 0.35 // coeeficient pour la partie proportionelle
#define KD 0 // coefficient pour la partie dérivée
unsigned long debut = millis();
int pos;
int pos2;
int DP;
unsigned long T;

int turn;

unsigned int dir_obs;
void setup() {
  
  motors_setup();
  InitEncoders();
  lineFollower_setup();

  // initialization of the serial communication.
  Serial.begin(9600);
  Serial.setTimeout(10);
  setMotorDVoltage(PWMT);
  setMotorGVoltage(PWMT);
}


void loop() {
  // Main loop
  if (millis() - T > DT){
    T = millis();
    dir_obs = turnDirection();
    lineFollower_loop();
    pos = positionRelative();
    DP = pos - pos2;
    pos2 = pos;

    Serial.print("Dstance: ");
    Serial.println(get_cm());

    //setMotorDVoltage(0);
    //setMotorGVoltage(0);

    if (pos > 0) {
      setMotorGVoltage(PWMT);
      setMotorDVoltage(PWMT - abs(pos) * KP + abs(DP / (DT*0.001)) * KD - abs(turn) * PWMT );
    }
    else if (pos < 0 ) {
      setMotorGVoltage(PWMT - abs(pos) * KP + abs(DP / (DT*0.001)) * KD - abs(turn) * PWMT);
      setMotorDVoltage(PWMT);
    }
    else {
      setMotorDVoltage(PWMT);
      setMotorGVoltage(PWMT);
    }



  }
  
  if (get_cm() < 15 && dir_obs) {
    T = millis();
    
    while (millis() - T < 600){
      if (dir_obs == 1){
        setMotorGVoltage(PWMT);
        setMotorDVoltage(-PWMT /2);
      }
      else {
        setMotorGVoltage(-PWMT /2);
        setMotorDVoltage(PWMT);
      }
      //if (getState != 15 && millis() - T > 100) {
      //  break;
      //  } 
    }
    while (getState() != 15){
      lineFollower_loop();
      pos = positionRelative();
      DP = pos - pos2;
      pos2 = pos;

      Serial.println("evado la wea");
      if (pos > 0) {
        setMotorGVoltage(PWMT);
        setMotorDVoltage(PWMT - abs(pos) * KP + abs(DP / (DT*0.001)) * KD - abs(turn) * PWMT );
      }
      else if (pos < 0 ) {
        setMotorGVoltage(PWMT - abs(pos) * KP + abs(DP / (DT*0.001)) * KD - abs(turn) * PWMT);
        setMotorDVoltage(PWMT);
      }
      else {
        setMotorDVoltage(PWMT);
        setMotorGVoltage(PWMT);
      }
    }
    
  }




}


void waitNextPeriod() 
{
  static long LastMillis=0; 
  long timeToWait = DT - ( millis() - LastMillis) ;
  if(timeToWait>0)
    delay(timeToWait);
  LastMillis=millis();
}
