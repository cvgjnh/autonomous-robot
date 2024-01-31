#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <const.h>
#include <pinSetup.h>
#include <Encoders.h>
#include <claw.h>
#include <sonar.h>
#include <tapeFollowing.h>
#include <IRFollowing.h>

enum IR_SENSOR
{
  LEFT_IR,
  RIGHT_IR
};

void Tape_following();
void IR_following();
void read_IR(IR_SENSOR);

Servo servoClaw;
Servo servoJoint;
Servo servoBase;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing backSonar(BACK_TRIG_PIN, BACK_ECHO_PIN, MAX_DISTANCE);

Servo servoLinkL;
Servo servoLinkR;
Tape Tape_follow(10, 1);

volatile uint32_t P_value = Tape_follow.P_value;
volatile uint32_t D_value = Tape_follow.D_value;
volatile int reflectanceL = 0;
volatile int reflectanceR = 0;
volatile int reflectanceLL = 0;
volatile int reflectanceRR = 0;
volatile int Left_RFSensor = 0;
volatile int Right_RFSensor = 0;
volatile int error = 0;
volatile int lasterr = 0;
volatile int G = 0;
volatile int RL_error = 0;
volatile int RR_error = 0;

const float soundc = 331.4 + (0.606 * TEMP) + (0.0124 * HUM);
const float soundcm = soundc / 100;
volatile int stage;
int treasure;
int curr_base_pos;

void setup()
{
  pinSetup();

  Claw::initializeClaw(&servoClaw);
  Claw::initializeBase(&servoBase);
  Claw::initializeJoint(&servoJoint);
  Sonar::initializeSonar(&sonar, &backSonar);

  Claw::clawSetup();

  stage = 1;
  treasure = 0; // Keeps track of which treasure we are on
  curr_base_pos = 90;
}

void loop()
{
  while (stage == 1) // Tape following stage
  {
    servoJoint.write(174); // Default position so that the ramp is not detected
    Tape_following();

    if (treasure == 0)
    {
      // Position the claw in the expected direction of the first treasure
      curr_base_pos = Claw::baseRotate(LEFTMOST + 2, Claw::base_servo_ptr->read());
      float dist = Sonar::getDist(soundcm);
      if (dist <= 20 && dist != 0)
      { // Object is close enough to pick up
        Tape::tp_motor_stop();
        Tape::bridge_Right();
        delay(80);
        Tape::tp_motor_stop();
        servoJoint.write(180);
        treasure = Sonar::detecting(soundcm, LEFTMOST, curr_base_pos, dist);
        Tape_follow.bridge_Back();
        delay(1000);
      }
    }
    else if (treasure == 1)
    {
      curr_base_pos = Claw::baseRotate(LEFTMOST + 2, Claw::base_servo_ptr->read());
      float dist = Sonar::getDist(soundcm);
      if (dist <= 20 && dist != 0)
      {
        Tape::tp_motor_stop();
        delay(100);
        Tape::bridge_Right();
        delay(10);
        Tape::tp_motor_stop();
        servoJoint.write(180);
        treasure = Sonar::detecting(soundcm, LEFTMOST, curr_base_pos, dist);
        Tape_follow.bridge_Back();
        delay(600);
      }
    }
    else
    {
      curr_base_pos = Claw::baseRotate(90, Claw::base_servo_ptr->read());
      Claw::closeClaw();
    }
  }

  while (stage == 2) // IR following stage
  {
    IR_following();
    Sonar::detecting(soundcm, LEFTMOST);
  }
}

/**
 * @brief Function for tape following.
 *
 * This function implements the logic for tape following using reflectance sensors.
 * It reads the sensor values, adjusts the motor speed and direction based on the sensor readings,
 * and performs specific actions based on certain conditions.
 */
void Tape_following()
{

  reflectanceL = analogRead(R_L_Sensor);
  reflectanceR = analogRead(R_R_Sensor);
  reflectanceLL = analogRead(R_L_Sensor_2);
  reflectanceRR = analogRead(R_R_Sensor_2);
  Serial.print("reflectance L: ");
  Serial.print(reflectanceL);
  Serial.print("  reflectance R: ");
  Serial.print(reflectanceR);
  Serial.print("  reflectance LL: ");
  Serial.print(reflectanceLL);
  Serial.print("  reflectance RR: ");
  Serial.print(reflectanceRR);
  Serial.println(" ");

  // If we detect the chicken wire, adjust the angle and go full speed to get through it
  while (reflectanceL > CW_Threshold && reflectanceR > CW_Threshold)
  {
    Tape_follow.tp_motor_stop();
    Tape_follow.tp_motor_right(40);
    reflectanceL = analogRead(R_L_Sensor);
    reflectanceR = analogRead(R_R_Sensor);
    error = 0;
    delay(180);
    Tape_follow.CW(1100);
    delay(400);
    Tape_follow.tp_motor_stop();
  }

  // If offtape, turn left or right to get back on tape
  while (reflectanceL <= OffTape_Threshold_X && reflectanceR <= OffTape_Threshold_X &&
         reflectanceLL <= Side_Threshold_L && reflectanceRR <= Side_Threshold_R)
  {
    Tape_follow.tp_motor_offtape();
    delay(50);
  }

  RL_error = reflectanceL - PID_Threshold_L;
  RR_error = reflectanceR - PID_Threshold_R;

  // If the left and right sensors detect both the tape, go straight
  if (RL_error > 0 && RR_error > 0)
  {
    error = 0;
    G = 0;
    Tape_follow.tp_motor_straight();
  }
  // If only the right sensor detects the tape, turn right
  else if (RL_error < 0 && RR_error > 0)
  {
    error = RL_error;
    G = Tape_follow.PID(P_value, D_value, error);
    Tape_follow.tp_motor_right(G);
  }
  // If only the left sensor detects the tape, turn left
  else if (RR_error < 0 && RL_error > 0)
  {
    error = abs(RR_error);
    G = Tape_follow.PID(P_value, D_value, error);
    Tape_follow.tp_motor_left(G);
  }
  else
  {
    // If only the far right sensor detects the tape, turn right sharply
    if (reflectanceRR > Side_Threshold_R)
    {
      error = -160;
      G = Tape_follow.PID(P_value, D_value, error);
      Tape_follow.tp_motor_right(G);
    }
    // If only the far left sensor detects the tape, turn left sharply
    else if (reflectanceLL > Side_Threshold_L)
    {
      error = 130;
      G = Tape_follow.PID(P_value, D_value, error);
      Tape_follow.tp_motor_left(G);
      // If both left sensors detect the tape, this is our cue to turn left into the archway
      if (reflectanceL > Archway_Threshold && reflectanceLL > Archway_Threshold)
      {
        // Turn left little by little until the right sensor detects the archway tape
        while (reflectanceRR < Archway_Threshold)
        {
          Tape_follow.tp_motor_stop();
          delay(100);
          Tape_follow.left_turn(300);
          delay(100);
          Tape_follow.tp_motor_stop();
          delay(50);
          reflectanceRR = analogRead(R_R_Sensor_2);
        }
        Tape_follow.tp_motor_straight();
        delay(300);
        if (stage == 1)
        {
          stage++;
        }
      }
    }
  }
}

/**
 * Function to control the robot's behavior during the IR stage.
 */
void IR_following()
{
  int i = 0;
  int Left_IR;
  int Right_IR;
  int IR_error;
  int G;
  int IR_P_value = 9;
  int IR_D_value = 2;

  // Read the IR sensors
  while (i < 2)
  {

    if (i % 2 == 0)
    {
      read_IR(LEFT_IR);
      Left_IR = analogRead(IR_Sensor);
    }
    else
    {
      read_IR(RIGHT_IR);
      Right_IR = analogRead(IR_Sensor);
    }

    i++;
  }

  IR_error = Left_IR - Right_IR;

  // If both sensors read similar values, go straight
  if (IR_error >= -IR_Threshold && IR_error <= IR_Threshold)
  {
    IR_error = 0;
    G = 0;
    Tape::tp_motor_straight();
  }
  // If the left sensor reads a higher value, turn left
  else if (IR_error > IR_Threshold)
  {
    G = Tape_follow.PID(IR_P_value, IR_D_value, IR_error);
    Tape::tp_motor_left(G);
  }
  // If the right sensor reads a higher value, turn right
  else if (IR_error < -IR_Threshold)
  {
    IR_error = abs(IR_error);
    G = Tape_follow.PID(IR_P_value, IR_D_value, IR_error);
    Tape::tp_motor_right(G);
  }
}

/**
 * Reads the infrared (IR) sensor and performs necessary actions.
 *
 * @param sensor The type of IR sensor to read (LEFT_IR or RIGHT_IR).
 */
void read_IR(IR_SENSOR sensor)
{
  int low_switch, high_switch;
  switch (sensor)
  {
  case LEFT_IR:
    low_switch = IR_Right_Switch;
    high_switch = IR_Left_Switch;
    break;
  case RIGHT_IR:
    low_switch = IR_Left_Switch;
    high_switch = IR_Right_Switch;
    break;
  }

  digitalWrite(low_switch, LOW);
  delay(50);
  digitalWrite(high_switch, HIGH);

  digitalWrite(IR_Discharge, HIGH);
  delay(3);
  digitalWrite(IR_Discharge, LOW);
  delay(3);
}