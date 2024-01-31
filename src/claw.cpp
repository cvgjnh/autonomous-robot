#ifndef CLAW_C
#define CLAW_C

#include <const.h>
#include <claw.h>

Servo *Claw::claw_servo_ptr;
Servo *Claw::joint_servo_ptr;
Servo *Claw::base_servo_ptr;
float Claw::rackPosition;
int Claw::treasureCount;

void Claw::initializeClaw(Servo *_claw_servo)
{
    Claw::claw_servo_ptr = _claw_servo;
}

void Claw::initializeJoint(Servo *_joint_servo)
{
    Claw::joint_servo_ptr = _joint_servo;
}

void Claw::initializeBase(Servo *_base_servo)
{
    Claw::base_servo_ptr = _base_servo;
}

/**
 * Initializes the claw setup by attaching servo motors and setting initial positions.
 * This function also initializes the rack position and treasure count variables.
 * It includes a delay of 4000 milliseconds to allow the servos to stabilize.
 */
void Claw::clawSetup()
{
    claw_servo_ptr->attach(SERVOCLAW);
    claw_servo_ptr->write(CLAWMAX);
    joint_servo_ptr->attach(SERVOJOINT);
    joint_servo_ptr->write(180);
    base_servo_ptr->attach(SERVOBASE);
    base_servo_ptr->write(90);
    rackPosition = 0;
    treasureCount = 0;
    delay(4000);
}

/**
 * Rotates the claw to the target position.
 *
 * @param base_target_pos The target position of the base servo.
 * @param base_current_pos The current position of the base servo.
 */
int Claw::baseRotate(int base_target_pos, int base_current_pos)
{
    if (base_current_pos < base_target_pos)
    {
        while (base_current_pos < base_target_pos)
        {
            base_servo_ptr->write(base_current_pos);
            base_current_pos++;
            delay(10);
        }
    }
    else if (base_current_pos > base_target_pos)
    {
        while (base_current_pos > base_target_pos)
        {
            base_servo_ptr->write(base_current_pos);
            base_current_pos--;
            delay(10);
        }
    }
    return base_current_pos;
}

/**
 * Sets the position of the claw joint based on the given state.
 *
 * @param state The state of the claw joint. Valid states are:
 *              - 1: Raised
 *              - 0: Lowered
 *              - 2: Zipline
 *              - 3: Default
 */
void Claw::clawJoint(int state)
{ // only 3 states: raised = 1, lowered = 0, zipline = 2, default = 3
    int joint_pos;
    if (state == 1)
    {
        for (joint_pos = 0; joint_pos < JOINTMAX; joint_pos += 1)
        {
            joint_servo_ptr->write(180 - joint_pos);
            delay(20);
        }
    }
    else if (state == 0)
    {
        for (joint_pos = JOINTMAX; joint_pos > 0; joint_pos -= 1)
        {
            joint_servo_ptr->write(180 - joint_pos);
            delay(10);
        }
    }
    else if (state == 3)
    {
        for (joint_pos = 0; joint_pos < 10; joint_pos += 1)
        {
            joint_servo_ptr->write(180 - joint_pos);
            delay(20);
        }
    }
    else
    {
        for (joint_pos = 0; joint_pos < 90; joint_pos += 1)
        {
            joint_servo_ptr->write(180 - joint_pos);
            delay(10);
        }
    }
}

/**
 * Moves the rack forward by the given distance.
 *
 * @param distancecm The distance to move the rack forward in centimeters.
 */
void Claw::ForwardStep(float distancecm)
{
    digitalWrite(dir, HIGH);
    int stepNum = distancecm * 340;
    for (int y = 0; y < stepNum; y++)
    {
        digitalWrite(stp, HIGH); // Trigger one step
        delay(1);
        digitalWrite(stp, LOW); // Pull step pin low so it can be triggered again
    }
    rackPosition += distancecm;
}

/**
 * Moves the rack backward by the given distance.
 *
 * @param distancecm The distance to move the rack backward in centimeters.
 */
void Claw::BackwardStep(float distancecm)
{
    digitalWrite(dir, LOW);
    int stepNum = distancecm * 340;
    for (int y = 0; y < stepNum; y++)
    {
        digitalWrite(stp, HIGH); // Trigger one step
        delay(1);
        digitalWrite(stp, LOW); // Pull step pin low so it can be triggered again
    }
    rackPosition -= distancecm;
}

/**
 * Moves the rack to the given destination.
 * Note that fully retracted is 0 and fully extended is 8.5.
 *
 * @param destinationcm The destination to move the rack to in centimeters.
 */
void Claw::moveRack(float destinationcm)
{
    float difference = destinationcm - rackPosition;
    if (difference > 0)
    {
        ForwardStep(difference);
    }
    else if (difference < 0)
    {
        BackwardStep(-difference);
    }
}

/**
 * Checks the state of the claw's hall sensor to determine if a bomb is present.
 *
 * @return 1 if a bomb is detected, 0 otherwise.
 */
int Claw::isBomb()
{
    int state = digitalRead(HALL);
    int bomb = 0;
    if (state == 0)
    { // bomb
        bomb = 1;
    }
    return bomb;
}

/**
 * @brief Closes the claw gradually until it reaches the minimum position.
 *
 * This function closes the claw gradually by decreasing the claw position until it reaches the minimum position.
 * It checks if there is a bomb present using the `isBomb()` function and sets the `safe` flag accordingly.
 * If a bomb is detected, the `safe` flag is set to 0, indicating that it is not safe to continue closing the claw.
 *
 * @return 1 if the claw was closed safely, 0 if a bomb was detected and the claw was stopped prematurely.
 */
int Claw::closeClaw()
{
    int safe = 1;
    for (int claw_pos = CLAWMAX; claw_pos > 0; claw_pos -= 1)
    {
        if (isBomb() == 1)
        {
            safe = 0;
        }

        if (safe == 0)
        {
            break;
        }
        claw_servo_ptr->write(claw_pos);
        delay(10);
    }
    return safe;
}

/**
 * Opens the claw by gradually increasing the position of the claw servo.
 */
void Claw::openClaw()
{
    for (int claw_pos = claw_servo_ptr->read(); claw_pos <= CLAWMAX; claw_pos += 1)
    {
        claw_servo_ptr->write(claw_pos);
        delay(10);
    }
}

/**
 * Picks up a treasure using the claw.
 * Used when the sonar sensor detects a treasure.
 *
 * @param current_base_pos The current angular position of the base servo.
 */
void Claw::clawPickUp(int current_base_pos)
{

    int safe = closeClaw();

    if (safe == 1)
    {

        current_base_pos = baseRotate(90, current_base_pos);

        // rack to depositing position
        moveRack(DEPOSITPOS);

        // raise joint
        clawJoint(1);

        // open claw to drop treasure
        openClaw();

        // lower joint
        clawJoint(0);

        // return to rack original position
        moveRack(RETRACTEDPOS);

        baseRotate(90, current_base_pos);

        treasureCount++;
    }
    else
    {
        openClaw();
        moveRack(RETRACTEDPOS);
        baseRotate(90, current_base_pos);
    }
}

#endif