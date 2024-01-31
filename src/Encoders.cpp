#include "const.h"
#include "Encoders.h"

// This file was used to experiment with using encoders.
// An encoder was attached to the left wheel of the robot to measure distance travelled.
// It is not used in the final code.

// 196 cm = 146 pulse

Encoders::Encoders()
{
  countL = 0;
}

void Encoders::handle_L_interrupt()
{
  countL++;
}

/**
 * Drives the robot with a specified speed until the left encoder count reaches the specified stop value.
 *
 * @param leftStop The stop value for the left encoder count.
 * @param speedL The speed at which the robot should move.
 */
void Encoders::drive(int leftStop, int speedL)
{
  countL = 0;

  // Robot starts moving with said speed
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, MOTOR_FREQ, speedL, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, speedL, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);

  bool leftDone = false;

  while (!leftDone) // reaches distance determined by encoder then stops
  {
    if (countL > leftStop && !leftDone)
    {
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
      leftDone = true;
    }
  }
}

/**
 * Turns the robot to the right until the left encoder count reaches the specified stop value.
 *
 * @param leftStop The stop value for the left encoder count.
 */
void Encoders::turnR(int leftStop)
{
  delay(100);

  countL = 0;

  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);

  bool leftDone = false;

  while (!leftDone)
  {
    if (countL > leftStop && !leftDone)
    {
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
      leftDone = true;
    }
  }
}

/**
 * This function moves the robot backwards at a specified speed until the left encoder count reaches a specified stop value.
 *
 * @param leftStop The stop value for the left encoder count.
 * @param speedL The speed at which the robot should move backwards.
 */
void Encoders::backup(int leftStop, int speedL)
{
  countL = 0;

  pwm_start(MOTOR_R_B, MOTOR_FREQ, speedL, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, speedL, RESOLUTION_12B_COMPARE_FORMAT);

  bool leftDone = false;

  while (!leftDone)
  {
    if (countL > leftStop && !leftDone)
    {
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
      leftDone = true;
    }
  }
}
