#ifndef CLAW_H
#define CLAW_H

#include <Servo.h>
class Claw
{
public:

    static Servo* claw_servo_ptr;
    static Servo* joint_servo_ptr;
    static Servo* base_servo_ptr;
    static float rackPosition;
    static int treasureCount;
    static void clawSetup();
    static void initializeClaw(Servo* _claw_servo);
    static void initializeJoint(Servo* _joint_servo);
    static void initializeBase(Servo* _base_servo);
    static void rotateZero(int base_current_pos);
    static int baseRotate(int base_target_pos, int base_current_pos);
    static void clawJoint(int state);
    static void clawPickUp(int base_pos);
    static int closeClaw();
    static void openClaw();
    static void ForwardStep(float distancecm);
    static void BackwardStep(float distancecm);
    static void moveRack(float destinationcm);
    static int getBasePos(int base_pos); //???
    static int isBomb();
};

#endif