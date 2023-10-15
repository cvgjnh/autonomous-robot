#ifndef LINKAGE_C
#define LINKAGE_C

#include "const.h"
#include <linkage.h>

Servo* Linkage::linkage_servo_ptr_1;
Servo* Linkage::linkage_servo_ptr_2;

void Linkage::initializeLink(Servo* linkage_servo1, Servo* linkage_servo2){
    Linkage::linkage_servo_ptr_1 = linkage_servo1;
    Linkage::linkage_servo_ptr_2 = linkage_servo2;
}

void Linkage::linkageSetup(){
    linkage_servo_ptr_1->attach(linkL);
    linkage_servo_ptr_2->attach(linkR);
    linkage_servo_ptr_1->write(0);
    linkage_servo_ptr_2->write(180);
    delay(2000);
}

void Linkage::liftBox(){

    int val = 135;
    //servo turns to lift box
    for (int pos = 0; pos <= val; pos += 1) { 
        // in steps of 1 degree
        linkage_servo_ptr_1->write(pos);
        linkage_servo_ptr_2->write(180-pos);
        delay(30);
    }
    delay(2000);
     for (int pos = val; pos >= 0; pos -= 1) { 
        linkage_servo_ptr_1->write(pos);
        linkage_servo_ptr_2->write(180-pos);
        delay(30);                       // waits 15ms for the servo to reach the position
    }
    delay(2000);
}

void Linkage::dropRamp(){

    //sonar detects the IR beacon in front of it

    //wheels turn to a position to drop the ramp
    //robot moves back until sonar detects the edge
    //drops ramp

    int push_ramp = 100;

    for (int posi = 0; posi <= push_ramp; posi += 1) { 
        // in steps of 1 degree
        linkage_servo_ptr_1->write(posi);
        linkage_servo_ptr_2->write(180-posi);
        delay(30);
    }
    delay(500);
     for (int posi = push_ramp; posi >= 0; posi -= 1) { 
        linkage_servo_ptr_1->write(posi);
        linkage_servo_ptr_2->write(180-posi);
        delay(30);                      
    }
    delay(500);

}



#endif