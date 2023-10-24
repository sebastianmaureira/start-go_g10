#include "Ultrasonic.h"
#include <MePort.h>



MeUltrasonicSensor ultraSensor(ULTRASENSOR);


int get_cm() {
  int salida = ultraSensor.distanceCm();
  return salida;
  }
