#ifndef TAPEFOLLOWING_C
#define TAPEFOLLOWING_C

#include <const.h>
#include <tapeFollowing.h>

Tape::Tape(int p_val, int d_val)
{
  P_value = p_val;
  D_value = d_val;
}

/**
 * Calculates the PID value for tape following.
 *
 * @param P_gain The proportional gain.
 * @param D_gain The derivative gain.
 * @param R The error value.
 * @return The calculated PID value.
 */
int Tape::PID(int P_gain, int D_gain, int R)
{
  int P;
  int D;
  int G;

  P = P_gain * abs(R);
  D = D_gain * (abs(R) - abs(lasterr));

  G = P + D;
  lasterr = R;
  return G / 13;
}

/**
 * Sets the motors to move straight forward.
 * This function starts the PWM signals for the right and left motors to move forward at a base speed.
 */
void Tape::tp_motor_straight()
{
  pwm_start(MOTOR_R_F, MOTOR_FREQ, BASE_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  pwm_start(MOTOR_L_F, MOTOR_FREQ, BASE_SPEED + 30, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

/**
 * Sets the left motor speed higher than the right based on the gain value.
 *
 * @param gain The gain value to adjust the motor speed.
 */
void Tape::tp_motor_left(int gain)
{
  pwm_start(MOTOR_R_F, MOTOR_FREQ, BASE_SPEED + gain, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  pwm_start(MOTOR_L_F, MOTOR_FREQ, 140, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

/**
 * Sets the right motor speed higher than the left based on the gain value.
 *
 * @param gain The gain value to adjust the motor speed.
 */
void Tape::tp_motor_right(int gain)
{
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 120, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  pwm_start(MOTOR_L_F, MOTOR_FREQ, BASE_SPEED + gain + 20, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

/**
 * Perform a left turn by controlling the motors.
 *
 * @param gain The gain value for the right motor forward movement.
 */
void Tape::left_turn(int gain)
{
  pwm_start(MOTOR_R_F, MOTOR_FREQ, gain, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 300, RESOLUTION_10B_COMPARE_FORMAT);
}

/**
 * Used when the robot is completely off the tape.
 * Based on the error value from memory, the robot will turn left or right to get back on the tape.
 */
void Tape::tp_motor_offtape()
{
  int G;
  G = 200;
  if (error > 0)
  {
    tp_motor_left(G);
  }
  else if (error < 0)
  {
    tp_motor_right(G + 30);
  }
  else
  {
    tp_motor_straight();
  }
}

/**
 * Stops the motors.
 */
void Tape::tp_motor_stop()
{
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

/**
 * Runs the motors at a specified speed forwards to cross the chicken wire.
 */
void Tape::CW(int speed)
{
  pwm_start(MOTOR_R_F, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, speed + 20, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

/**
 * Performs a right turn at a specified speed.
 */
void Tape::bridge_Right()
{
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 500, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 500, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

/**
 * Starts the motors to move backwards.
 */
void Tape::bridge_Back()
{
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 500, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 500, RESOLUTION_10B_COMPARE_FORMAT);
}

#endif