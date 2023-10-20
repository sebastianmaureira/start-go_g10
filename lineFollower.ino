#include "lineFollower.h"
#include "MeRGBLineFollower.h"
#include <MePort.h>


MeRGBLineFollower RGBLineFollower(LINEFOLLOWER, ADDRESS2);


T_octet lfSensor; // octet contenant l'information Noir/Blanc
// dans ses bits de poids faible 
// 0000 00 S1 S2

// Capteur array :
// 00 S6 S5 S4 S3 S2 S1  

// Capteur RGB : 
// 0000 S4 S3 S2 S1  

// NB : la librairie propose un algo de fusion de données 
// pour déduire l'angle à appliquer au robot
// Elle permet d'accéder aux informations de chaque capteur par 
// RGBLineFollower.getADCValueRGB1()
// Ainsi qu'à une information binaire (blanc/noir) par RGBLineFollower.getPositionState()
// Cf. http://docs.makeblock.com/diy-platform/en/electronic-modules/sensors/rgb-line-follower.html



uint8_t bits_pos;
int last_dir;
// last_dir 0  str8
// last_dir 1  droite
// last_dir 2 gauche

void lineFollower_loop() {

  RGBLineFollower.loop();
  lfSensor = RGBLineFollower.getPositionOffset();     


bits_pos = RGBLineFollower.getPositionState();
turn=0;
if (bits_pos == 15){
  if (last_dir == 1){
    lfSensor = -100;
    turn = -1;
    }
  else{
   lfSensor = 100;
   turn = 1;
        }
}
else if (bits_pos==0){
        lfSensor = 0;
        }
else if (bits_pos<8){
        last_dir = 2;
        }
else if (bits_pos > 7){
        last_dir = 1;
        }
Serial.println(lfSensor);
Serial.println(turn);
}


void lineFollower_setup() {

    RGBLineFollower.begin();

}

int positionRelative() {
  return lfSensor;
}




