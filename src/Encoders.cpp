#include "const.h"
#include "Encoders.h"

//196 cm = 146 pulse

Encoders::Encoders()
{
  countL = 0;
  //countR = 0;
}

void Encoders::handle_L_interrupt()
{
  countL++;
}

// void Encoders::handle_R_interrupt()
// {
//   countR++;
// }

// void Encoders::drive(int leftStop, int rightStop, int speedL, int speedR)
// {
//   countL = 0;
//   countR = 0;

//   //Robot starts moving with said speed
//   pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_R_F, MOTOR_FREQ, speedR, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_L_F, MOTOR_FREQ, speedL, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);

//   bool rightDone = false;
//   bool leftDone = false;

//   while (!rightDone || !leftDone) // reaches distance determined by encoder then stops
//   {
//     if (countR > rightStop && !rightDone)
//     {
//       pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       rightDone = true;
//     }

//     if (countL > leftStop && !leftDone)
//     {
//       pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       leftDone = true;
//     }
//   }
// }

void Encoders::drive(int leftStop, int speedL)
{
  countL = 0;

  //Robot starts moving with said speed
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

// void Encoders::turnR(int leftStop, int rightStop)
// {
//   delay(100);

//   countL = 0;
//   countR = 0;

//   pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_R_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_L_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);

//   bool rightDone = false;
//   bool leftDone = false;

//   while (!rightDone || !leftDone)
//   {
//     if (countR > rightStop && !rightDone)
//     {
//       pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       rightDone = true;
//     }
//     if (countL > leftStop && !leftDone)
//     {
//       pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       leftDone = true;
//     }  
//   }
// }

// void Encoders::turnL(int leftStop, int rightStop)
// {
//   delay(100);

//   countL = 0;
//   countR = 0;

//   pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_R_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_L_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);

//   bool rightDone = false;
//   bool leftDone = false;

//   while (!rightDone || !leftDone)
//   {
//     if (countR > rightStop && !rightDone)
//     {
//       pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       rightDone = true;
//     }
//     if (countL > leftStop && !leftDone)
//     {
//       pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       leftDone = true;
//     }
//   }
// }

// void Encoders::backup(int leftStop, int rightStop, int speedL, int speedR)
// {
//   countL = 0;
//   countR = 0;

//   pwm_start(MOTOR_R_B, MOTOR_FREQ, speedR, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//   pwm_start(MOTOR_L_B, MOTOR_FREQ, speedL, RESOLUTION_12B_COMPARE_FORMAT);

//   bool rightDone = false;
//   bool leftDone = false;

//   while (!rightDone || !leftDone)
//   {
//     if (countR > rightStop && !rightDone)
//     {
//       pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       rightDone = true;
//     }

//     if (countL > leftStop && !leftDone)
//     {
//       pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//       leftDone = true;
//     }
//   }
// }

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

void Encoders::rightPivot()
{
  turnR(23);
}
void Encoders::rightPivotCount(int counts) 
{
  turnR(counts);
}

void Encoders::adjustmentBackup()
{
  backup(14, ENC_STRAIGHT_SPEED);
}

void Encoders::adjustmentBackupCount(int counts)
{
  backup(counts, ENC_STRAIGHT_SPEED);
}