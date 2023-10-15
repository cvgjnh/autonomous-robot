#ifndef SONAR_H
#define SONAR_H

#include "const.h"
#include "claw.h"

class Sonar
{
public:
    static NewPing* sonar_ptr;
    static NewPing* back_sonar_ptr;
    static void initializeSonar(NewPing* _sonar, NewPing* _backsonar);
    static float getDist(float soundcm);
    static int detecting(float soundcm, int targ_base_pos, int base_pos, int distance);
    static int yesTreasure;
};

#endif
