#ifndef TAPEFOLLOWING_C
#define TAPEFOLLOWING_C

#include <const.h>
#include <tapeFollowing.h>

//setup

Tape::Tape(int p_val, int d_val){
  P_value = p_val;

  D_value = d_val;
}

//get gain
///
int Tape:: PID (int P_gain, int D_gain, int R) {
  int P;
  int D;
  int G;
  
  P=P_gain*abs(R);
  D=D_gain*(abs(R)-abs(lasterr));

  G=P+D;
  lasterr=R;
  return G/13;
}

void Tape::tp_motor_straight() {
  pwm_start(MOTOR_R_F, MOTOR_FREQ, BASE_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
 
  pwm_start(MOTOR_L_F, MOTOR_FREQ, BASE_SPEED + 30, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

void Tape::tp_motor_left(int gain) {
  pwm_start(MOTOR_R_F, MOTOR_FREQ, BASE_SPEED + gain, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
 
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 140, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

void Tape::tp_motor_right(int gain) {
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 120, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
 
  pwm_start(MOTOR_L_F, MOTOR_FREQ, BASE_SPEED + gain+20, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

void Tape::left_turn(int gain) {
  pwm_start(MOTOR_R_F, MOTOR_FREQ, gain, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
 
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 300, RESOLUTION_10B_COMPARE_FORMAT);
}

void Tape::tp_motor_offtape() {
  int G;
  G=200;
  if (error > 0) {
    tp_motor_left(G);
   //Serial.println("left");
  }
  else if (error < 0) {
    tp_motor_right(G+30);
    //Serial.println("right");
  }
  else {
    tp_motor_straight();
    //Serial.println("straight");
  }

  //Serial.println(G);
  
}
void Tape::tp_motor_stop() {
pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
 
pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

void Tape::turnR(int leftStop)
{
  delay(100);

  interrupt_counter = 0;

  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 300, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 300, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  bool leftDone = false;

  while (!leftDone)
  {
    if (interrupt_counter > leftStop && !leftDone)
    {
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      leftDone = true;
    }
  }
}
void Tape::bridge_Left() {
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 500, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 500, RESOLUTION_10B_COMPARE_FORMAT);

}
void Tape::CW(int speed) {
  pwm_start(MOTOR_R_F, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, speed+20, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

}

void Tape::bridge_Right() {
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 500, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 500, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}
void Tape::bridge_Back() {
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 500, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 500, RESOLUTION_10B_COMPARE_FORMAT);
}

void Tape::handle_interrupt() {
  interrupt_counter++;
}

void Tape::tape_following() {
  
}

#endif