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


#define DT 40 //sampling period in milliseconds
#define PWMT 85 //définit le pwm de travail, celui d'un robot parfaitement calibré
#define KP 0.4// coeeficient pour la partie proportionelle
#define KI 0.11// coefficient pour la partie integrée
#define KD 0.04// coefficient pour la partie dérivée
#define KP2 0.6 // coeeficient pour la partie proportionelle
#define KI2 0.5 // coefficient pour la partie integrée
#define KD2 0 // coefficient pour la partie dérivée
unsigned long debut = millis();
int pos;
int pos2;
int DP;
unsigned long T;
float integrale = 0; 


int turn;
unsigned long T2;

unsigned int dir_obs;
int old_dir_obs;


void setup() {
  
  motors_setup();
  InitEncoders();
  lineFollower_setup();

  // initialization of the serial communication.
  Serial.begin(9600);
  Serial.setTimeout(10);
  setMotorDVoltage(PWMT);
  setMotorGVoltage(PWMT);
  T2 = 0;
  old_dir_obs = 0;
}


void loop() {
  // Main loop
  if (millis() - T2 > 500){
    T2 = millis();

    Serial.print("posicion: ");
    Serial.println(pos);

    Serial.print("integral: ");
    Serial.println(integrale);

    Serial.print("turn: ");
    Serial.println(turn);

    }
  if (millis() - T > DT){
    T = millis();
    dir_obs = turnDirection();
    lineFollower_loop();
    pos = positionRelative();
    DP = pos - pos2;
    pos2 = pos;
    integrale = integrale + (pos * DT * 0.001);
    if (abs(integrale)>700){
      if (integrale < 0){
        integrale = -700.0;
      }
      else{
        integrale = 700.0;
      }
    }
//
//    Serial.print("posicion: ");
//    Serial.println(pos);
//
//    Serial.print("integral: ");
//    Serial.println(integrale);

    //setMotorDVoltage(0);
    //setMotorGVoltage(0);

    if (pos > 0) {
      setMotorGVoltage(PWMT + (KP * abs(pos) + integrale*KI -abs(DP / (DT*0.001)) * KD)*0.7);
      setMotorDVoltage(PWMT - KP * abs(pos)  - integrale * KI  - abs(turn) * PWMT ); // + abs(DP / (DT*0.001)) * KD
    } 
    else if (pos < 0 ) {
      setMotorGVoltage(PWMT - abs(pos) * KP + integrale * KI  - abs(turn) * PWMT );  // + abs(DP / (DT*0.001)) * KD
      setMotorDVoltage(PWMT  + (abs(pos) * KP-  integrale * KI - abs(DP / (DT*0.001)) * KD) * 0.7);
    }
    else {
      setMotorDVoltage(PWMT * 1.2);
      setMotorGVoltage(PWMT * 1.2);
      // integrale = 0;
    }



  }
  
  if (get_cm() < 15 && dir_obs) {
    T = millis();
    old_dir_obs = dir_obs;
    while (millis() - T < 600){
      if (dir_obs == 1){
        setMotorGVoltage(PWMT * 0.7);
        setMotorDVoltage(-PWMT  /2);
      }
      else {
        setMotorGVoltage(-PWMT / 2);
        setMotorDVoltage(PWMT * 0.7);
      }
      //if (getState != 15 && millis() - T > 100) {
      //  break;
      //  } 
    }
    integrale = 0;
    while (not getState() & 9){
      if (millis() - T > DT){
        T = millis();
        lineFollower_loop();
        pos = positionRelative();
        DP = pos - pos2;
        pos2 = pos;
        integrale = integrale + pos * DT * 0.001;
        Serial.println("evado la wea");
        if (pos > 0) {
          setMotorGVoltage(PWMT*.7 +  integrale * KI2 / 2);
          setMotorDVoltage(PWMT*.7 - abs(pos) * KP2 - integrale * KI2 + abs(DP / (DT*0.001)) * KD2);// - abs(turn) * PWMT );
        }
        else if (pos < 0 ) {
          setMotorGVoltage(PWMT*.7 - abs(pos) * KP2 + integrale * KI2 + abs(DP / (DT*0.001)) * KD2);// - abs(turn) * PWMT);
          setMotorDVoltage(PWMT*.7 -  integrale * KI2 / 2);
        }
        else {
          setMotorDVoltage(PWMT / 2);
          setMotorGVoltage(PWMT / 2);
        }
      }
    }
    while (millis() - T < 400){
      if (old_dir_obs == 1){
        setMotorDVoltage(PWMT * 0.7);
        setMotorGVoltage(-PWMT  /2);
      }
      else {
        setMotorDVoltage(-PWMT / 2);
        setMotorGVoltage(PWMT * 0.7);
      }
      //if (getState != 15 && millis() - T > 100) {
      //  break;
      //  } 
    }
    integrale = 0;
    
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
