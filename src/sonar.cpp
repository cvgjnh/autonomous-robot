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

/**
 * Calculates the distance based on the duration of the sonar ping and the speed of sound.
 * 
 * @param soundcm The speed of sound in centimeters per second.
 * @return The calculated distance in centimeters.
 */
float Sonar::getDist(float soundcm){
    float dur = sonar_ptr->ping_median(ITERATIONS);
    float dist = (dur/200)*soundcm;
    return dist;
}

/**
 * Detects the treasure using the sonar sensor.
 * 
 * @param soundcm The speed of sound in centimeters per second.
 * @param targ_base_pos The estimated angular position of the base required to get the treasure.
 * @param base_pos The current angular position of the base.
 * @param distance The distance measured by the sonar sensor.
 * @return Returns 1 if the treasure is detected and picked up, 0 otherwise.
 */
int Sonar::detecting(float soundcm, int targ_base_pos, int base_pos, int distance) { 

    int distanceL=-1, distanceR=-1, distanceL2=-1, distanceR2=-1;

    const int targL = targ_base_pos - ANGLE;
    const int targR = targ_base_pos + ANGLE;
    
    // If possible, take a distance measurement to the left and right of the expected target position.
    if (targL >= LEFTMOST) {
        base_pos = Claw::baseRotate(targL, base_pos);
        distanceL = getDist(soundcm);
    }

    if (targR <= RIGHTMOST) {
        base_pos = Claw::baseRotate(targR, base_pos);
        distanceR = getDist(soundcm);
    }

    // By comparing the distance measurements, we can determine if the treasure is present.
    // The treasure is narrow so if the distance measurements are too close, we have detected a wall.
    // If a treasure is detected, we can pick it up.
    if(((distanceL-distance)>= 3 && (distanceR-distance >= 3)) || 
    ((distanceL-distance) >= 3 && (distanceR == -1)) ||
    ((distanceR-distance) >= 3 && (distanceL == -1))) { 

        int treasure_pos = targ_base_pos;
        Claw::baseRotate(treasure_pos,Claw::base_servo_ptr->read());
        Claw::moveRack(EXTENDEDPOS);
        Claw::clawPickUp(treasure_pos); 

        return true;
    }
  
    return false;

}

#endif