#include "motors.h"


void motors_setup()
{
  pinMode(MDBI1, OUTPUT);
  pinMode(MDBI2, OUTPUT);
  pinMode(MDPWM, OUTPUT);
  pinMode(MGBI1, OUTPUT);
  pinMode(MGBI2, OUTPUT);
  pinMode(MGPWM, OUTPUT);
}


// fonction permettant de gerer l'alimentation moteur (sens et amplitude)
// positif <=> avancer
void setMotorDVoltage(int valeur)
{
  if (valeur > 0)
  {
    digitalWrite(MDBI1, 1);
    digitalWrite(MDBI2, 0);
  }
  else
  {
    digitalWrite(MDBI1, 0);
    digitalWrite(MDBI2, 1);
  }
  analogWrite(MDPWM, constrain(abs(valeur), 0, MAXPWM));
}


// fonction permettant de gerer l'alimentation moteur (sens et amplitude)
void setMotorGVoltage(int valeur)
{
  if (valeur > 0)
  {
    // On inverse car le moteur est attaché dans l'autre sens
    digitalWrite(MGBI1, 0);
    digitalWrite(MGBI2, 1);
  }
  else
  {
    digitalWrite(MGBI1, 1);
    digitalWrite(MGBI2, 0);
  }
  analogWrite(MGPWM, constrain(abs(valeur), 0, MAXPWM));
}
