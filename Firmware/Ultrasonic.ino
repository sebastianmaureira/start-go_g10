#include "Ultrasonic.h"
#include <MePort.h>



MeUltrasonicSensor ultraSensor_front(ULTRASENSORFRONT);
MeUltrasonicSensor ultraSensor_side(ULTRASENSORSIDE);


unsigned int get_cm_front() {
  unsigned int salida = ultraSensor_front.distanceCm();
  return salida;
  }

unsigned int get_cm_side() {
  unsigned int salida = ultraSensor_side.distanceCm();
  return salida;
  }
