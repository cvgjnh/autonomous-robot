#ifndef IRFOLLOWING_H
#define IRFOLLOWING_H

#define IR_Threshold 50



class IR
{
public: 
  int PID (int P_gain, int D_gain, int R);
  enum IR_SENSOR {LEFT_IR, RIGHT_IR};

  void read_Left_IR();
  void read_Right_IR();
  void IR_Run();
  void read_IR(IR_SENSOR);

  volatile uint32_t IR_P_value = 9;
  volatile uint32_t IR_D_value = 2;
  volatile int Left_IR = 0;
  volatile int Right_IR = 0;
  volatile int IR_error = 0;
  volatile int IR_lasterr = 0;

};

#endif
