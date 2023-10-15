#ifndef SONAR_C
#define SONAR_C

#include <const.h>
#include <sonar.h>
#include <claw.h>
#include <tapeFollowing.h>
#include <NewPing.h>

NewPing* Sonar::sonar_ptr;
NewPing* Sonar::back_sonar_ptr;
int Sonar::yesTreasure = 0;

void Sonar::initializeSonar(NewPing* _sonar, NewPing* _backsonar){
    Sonar::sonar_ptr = _sonar;
    Sonar::back_sonar_ptr = _backsonar;
}

//
float Sonar::getDist(float soundcm){
    float dur = sonar_ptr->ping_median(ITERATIONS);
    float dist = (dur/200)*soundcm;
    return dist;
}

int Sonar::detecting(float soundcm, int targ_base_pos, int base_pos, int distance) { //target base position is the estimated position
// the arm should be in to get the treasure

     int distanceL=-1, distanceR=-1, distanceL2=-1, distanceR2=-1;
     // int curr_base_pos= getBasePos();
     const int targL = targ_base_pos - ANGLE;
     const int targR = targ_base_pos + ANGLE;
     
        //int base_pos = Claw::baseRotate(targ_base_pos, Claw::base_servo_ptr->read());
        //distance = getDist(soundcm);

        if (distance < 20 && distance != 0){
            //Robot stops moving (wheel speed zero) 
            //Tape::tp_motor_stop();

            if (targL>=LEFTMOST) {
                base_pos = Claw::baseRotate(targL, base_pos);
                distanceL = getDist(soundcm);
            }

            // if (targ_base_pos-10>=LEFTMOST) {
            //     base_pos = Claw::baseRotate(targ_base_pos-10, base_pos);
            //     distanceL2 = getDist(soundcm);
            // }

            // if (targ_base_pos+10<=RIGHTMOST)  {
            //     base_pos = Claw::baseRotate(targ_base_pos+10, base_pos);
            //     distanceR2 = getDist(soundcm);
            // }

            if (targR<=RIGHTMOST) {
                base_pos = Claw::baseRotate(targR, base_pos);
                distanceR = getDist(soundcm);
            }

            if(((distanceL-distance)>= 3 && (distanceR-distance >= 3)) || 
            ((distanceL-distance) >= 3 && (distanceR == -1)) ||
            ((distanceR-distance) >= 3 && (distanceL == -1))) { 
                //if( abs(distance-distanceL2)<=5 || abs(distance-distanceR2)<=5){ 
                // rack extends until the IR sensor no longer detects a signal as the idle is between the claw
                // check the hall effect sensor
                int treasure_pos = targ_base_pos;
                Claw::baseRotate(treasure_pos,Claw::base_servo_ptr->read());
                    Claw::moveRack(EXTENDEDPOS);
                    Claw::clawPickUp(treasure_pos); 

            
                //else{}
                yesTreasure=1;
            }
            else{

            }
        }
        return yesTreasure;

}

#endif