// PIN utilisées pour l'alimentation moteur

// Droit
#define MDPWM 12
#define MDBI1 34
#define MDBI2 35

// Gauche
#define MGPWM 8
#define MGBI1 37
#define MGBI2 36

#define MAXPWM 190 // maximum duty cycle for the PWM is 255/MAXPWM

void motors_setup();
void setMotorDVoltage(int value);
void setMotorGVoltage(int value);