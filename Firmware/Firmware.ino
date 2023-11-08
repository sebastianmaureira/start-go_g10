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


#define DT 45 //sampling period in milliseconds
#define DT_Filter 20
#define PWMT 92 //définit le pwm de travail, celui d'un robot parfaitement calibré
#define PWMT_OBS 40 //définit le pwm de travail, celui d'un robot parfaitement calibré
#define KP 0.3// coeeficient pour la partie proportionelle  .37    .35    .3    32
#define KI 0.11// coefficient pour la partie integrée        .13    .11   .12     11
#define KD 0.02// coefficient pour la partie dérivée         .03    .02  .025    3
#define KP2 0.3 // coeeficient pour la partie proportionelle
#define KI2 0.05 // coefficient pour la partie integrée
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
int avg;

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
  avg = 0;;
}


void loop() {
  // Main loop

/*
  if (millis() - T > DT_Filter){
    lineFollower_loop();
    avg = 0.45 *avg + 0.5 * positionRelative();
  }*/
  
  if (millis() - T > DT){
    T = millis();
    dir_obs = turnDirection(dir_obs);
    lineFollower_loop();
    pos = positionRelative();
    //pos = avg;
    DP = pos - pos2;
    pos2 = pos;
    integrale = integrale + (pos * DT * 0.001);
    if (abs(integrale)>200){
      if (integrale < 0){
        integrale = -200.0;
      }
      else{
        integrale = 200.0;
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
      setMotorDVoltage((PWMT - KP * abs(pos)  - integrale * KI) * (1- abs(turn))- abs(turn)*PWMT *0.75); // + abs(DP / (DT*0.001)) * KD
    } 
    else if (pos < 0 ) {
      setMotorGVoltage((PWMT - abs(pos) * KP + integrale * KI) * (1- abs(turn)) - abs(turn)*PWMT *0.75);  // + abs(DP / (DT*0.001)) * KD
      setMotorDVoltage(PWMT  + (abs(pos) * KP-  integrale * KI - abs(DP / (DT*0.001)) * KD) * 0.7);
    }
    else if(abs(integrale) < 10){
      setMotorDVoltage(PWMT*1.2);
      setMotorGVoltage(PWMT*1.2);
      // integrale = 0;
    }
    else {
      setMotorDVoltage(PWMT*1.1);
      setMotorGVoltage(PWMT*1.1);
      //Serial.println(abs(integrale));
      // integrale = 0;
    }
   


  //Serial.println(bp());
  //Serial.println(dir_obs);
  }
  
  if (get_cm() < 12 && dir_obs) {
    T = millis();
    old_dir_obs = dir_obs;
    while (millis() - T < 600){
      if (dir_obs == 1){
        setMotorGVoltage(1.3 * PWMT_OBS);
        setMotorDVoltage(-PWMT_OBS);
      }
      else {
        setMotorGVoltage(-PWMT_OBS);
        setMotorDVoltage(1.3 * PWMT_OBS);
      }
      //if (getState != 15 && millis() - T > 100) {
      //  break;
      //  } 
    }
    integrale = 0;
    while ((15 ^getState()) & 9){
      if (millis() - T > DT){
        T = millis();
        lineFollower_loop();
        pos = positionRelative();
        DP = pos - pos2;
        pos2 = pos;
        integrale = integrale + pos * DT * 0.001;
        //Serial.println("evado la wea");
        if (pos > 0) {
          setMotorGVoltage(PWMT_OBS*.7 +  integrale * KI2 / 2);
          setMotorDVoltage(PWMT_OBS*.7 - abs(pos) * KP2 - integrale * KI2 + abs(DP / (DT*0.001)) * KD2);// - abs(turn) * PWMT );
        }
        else if (pos < 0 ) {
          setMotorGVoltage(PWMT_OBS*.7 - abs(pos) * KP2 + integrale * KI2 + abs(DP / (DT*0.001)) * KD2);// - abs(turn) * PWMT);
          setMotorDVoltage(PWMT_OBS*.7 -  integrale * KI2 / 2);
        }
        else {
          setMotorDVoltage(PWMT_OBS*.9);
          setMotorGVoltage(PWMT_OBS*.9);
        }
      }
    }
    while (millis() - T < 400){
      if (old_dir_obs == 1){
        setMotorDVoltage(PWMT_OBS );
        setMotorGVoltage(-PWMT_OBS );
      }
      else {
        setMotorDVoltage(-PWMT_OBS);
        setMotorGVoltage(PWMT_OBS );
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
