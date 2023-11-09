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
#define PWMT 85 //définit le pwm de travail, celui d'un robot parfaitement calibré
#define PWMT_OBS 44 //définit le pwm de travail, celui d'un robot parfaitement calibré
#define KP 0.3// coeeficient pour la partie proportionelle  .37    .35    .3    3
#define KI 0.12// coefficient pour la partie integrée        .13    .11   .12     11
#define KD 0// coefficient pour la partie dérivée         .03    .02  .025    0
#define KP2 0.4 // coeeficient pour la partie proportionelle
#define KI2 0.07 // coefficient pour la partie integrée
#define KD2 0 // coefficient pour la partie dérivée
unsigned long debut = millis();
int pos;
int pos2;
int DP;
unsigned long T;
float integrale = 0; 

unsigned int state;
// state = 0 -> suivi ligne
// state = 1 -> turn
// state = 2 -> obstacle
int turn;
unsigned long T2;

unsigned int dir_obs;
int old_dir_obs;

unsigned int distance_front;
unsigned int distance_side;

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
  state = 0;
}


void loop() {
  // Main loop

  if (millis() - T > DT){
    T = millis();
    lineFollower_loop();
    dir_obs = turnDirection(dir_obs);
    pos = positionRelative();
    //pos = avg;
    DP = pos - pos2;
    pos2 = pos;
    integrale = integrale + (pos * DT * 0.001);

    distance_front = get_cm_front();
    distance_side = get_cm_side();

    // Turning state
    if (turn != 0){
      state = 1;
      integrale = 0;
      DP = 0;
      }
    else{
      state = 0;
    }

    // Saturate the integral
    if (abs(integrale)>500){
      if (integrale < 0){
        integrale = -500.0;
      }
      else{
        integrale = 500.0;
      }
    }

    // Turning avec PID AND turning state. (if turning state, no pid, just turns)
    if (distance_front > 16  and distance_side >15){
      if (pos > 0) {
        setMotorGVoltage(PWMT -abs(pos) *0.35 + (KP * abs(pos) + integrale * KI -abs(DP / (DT*0.001)) * KD)*0.2);   // -abs(integrale) * 0.2
        setMotorDVoltage((PWMT -abs(pos) *0.35 - KP * abs(pos)  - integrale * KI) * (1- abs(turn))- abs(turn)*PWMT *0.7); // + abs(DP / (DT*0.001)) * KD   -abs(integrale) * 0.2
      } 
      else if (pos < 0 ) {
        setMotorGVoltage((PWMT -abs(pos) *0.35- abs(pos) * KP + integrale * KI) * (1- abs(turn)) - abs(turn)*PWMT *0.2);  // + abs(DP / (DT*0.001)) * KD   -abs(integrale) * 0.2
        setMotorDVoltage(PWMT -abs(pos) *0.35 + (abs(pos) * KP- integrale * KI - abs(DP / (DT*0.001)) * KD)*0.7);    // -abs(integrale) * 0.2
      }
      // If straight AND no error, increase speed
      else if(abs(integrale) < 10){
        setMotorDVoltage(PWMT*1.6);
        setMotorGVoltage(PWMT*1.6);
        // integrale = 0;
      }
      // If straight, increase speed
      else {
        setMotorDVoltage(PWMT*1.2);
        setMotorGVoltage(PWMT*1.2);
        //Serial.println(abs(integrale));
        // integrale = 0;
      }
    } 


  // Turning logic
  else if (distance_front < 15 && distance_side > 10) {
    T = millis();
    old_dir_obs = dir_obs;
    // Forces 90º turn
    while (millis() - T < 580){
      if (dir_obs == 1){
        setMotorGVoltage(1.15 * PWMT_OBS);
        setMotorDVoltage(- (1.1*PWMT_OBS));
      }
      else {
        setMotorGVoltage(-(1.1*PWMT_OBS));
        setMotorDVoltage(1.15 * PWMT_OBS);
      }
      //if ((getState() ^ 15) & 6){
      //  break;
      //  }
    }

    
    integrale = 0;
    T2=millis();
    T = millis();
    // Follow the line at least a little.
    while (((15 ^ getState()) & 15 or millis() - T2 < 500) and T -T2 < 4500){
      if (millis() - T > DT){
        T = millis();
        lineFollower_loop();
        pos = positionRelative();
        DP = pos - pos2;
        pos2 = pos;
        integrale = integrale + pos * DT * 0.001;
        if (pos > 0) {
          setMotorGVoltage(PWMT_OBS*1.1);
          setMotorDVoltage(PWMT_OBS*1.1 - abs(pos) * KP2 - integrale * KI2 + abs(DP / (DT*0.001)) * KD2);  // - abs(turn) * PWMT );
        }
        else if (pos < 0 ) {
          setMotorGVoltage(PWMT_OBS*1.1 - abs(pos) * KP2 + integrale * KI2 + abs(DP / (DT*0.001)) * KD2);  // - abs(turn) * PWMT);
          setMotorDVoltage(PWMT_OBS*1.1);
        }
        else {
          setMotorDVoltage(PWMT_OBS*1.15);
          setMotorGVoltage(PWMT_OBS*1.15);
        }
      }
    }
    // Go back to OG line
    while (millis() - T < 50){
      if (dir_obs == 1){
        setMotorGVoltage(1.15 * PWMT_OBS);
        setMotorDVoltage(- (1.1*PWMT_OBS));
      }
      else {
        setMotorGVoltage(-(1.1*PWMT_OBS));
        setMotorDVoltage(1.15 * PWMT_OBS);
      }
      //if ((getState() ^ 15) & 6){
      //  break;
      //  }
    }
    integrale = 0;
    state = 0;
  }

  else if (distance_front < 15 and distance_side < 10){
      setMotorDVoltage(0);
      setMotorGVoltage(0);
      delay(500);
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
