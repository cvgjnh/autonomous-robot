#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <const.h>
#include <pinSetup.h>
#include <Encoders.h>
#include <claw.h>
#include <sonar.h>
#include <linkage.h>
#include <tapeFollowing.h>
#include <IRFollowing.h>

enum IR_SENSOR {LEFT_IR, RIGHT_IR};

void handle_L_interrupt();
void handle_interrupt();
void Tape_following();
void IR_following();
void read_IR(IR_SENSOR);

//Encoders encoders1 = Encoders();

Servo servoClaw;
Servo servoJoint;
Servo servoBase;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing backSonar(BACK_TRIG_PIN,BACK_ECHO_PIN,MAX_DISTANCE);

Servo servoLinkL;
Servo servoLinkR;
Tape Tape_follow (10,1);

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

void setup() {
  pinSetup();
  //Serial.begin(9600);
  // attachInterrupt(digitalPinToInterrupt(enc_L), handle_L_interrupt, FALLING);

  Claw::initializeClaw(&servoClaw);
  Claw::initializeBase(&servoBase);
  Claw::initializeJoint(&servoJoint);
  Sonar::initializeSonar(&sonar, &backSonar);
  //Linkage::initializeLink(&servoLinkL, &servoLinkR);

  Claw::clawSetup();
  //Linkage::linkageSetup();


  attachInterrupt(digitalPinToInterrupt(enc_L), handle_interrupt, RISING);

  
  //pwm_start(MOTOR_L_F, PWMFREQ, FWD_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  //pwm_start(MOTOR_R_F, PWMFREQ, FWD_SPEED, RESOLUTION_10B_COMPARE_FORMAT);

  //Serial.println("Serial start");
  stage = 1;
  treasure = 0;
  curr_base_pos = 90;

}

// void loop(){
//   int countR = 0;
//   int value = 0;

//   countR = encoders1.countR;

//   Serial.print("countR:");
//   Serial.println(countR,DEC);

//   delay(200);
  
//   }

// void loop() {
//   Claw::baseRotate(150,90);
//   Claw::baseRotate(30,150);
//   Claw::ForwardStep(9);
//   Claw::clawPickUp(30);

// }

void loop() {
  // float distance = Sonar::detecting(soundcm, LEFTMOST);
  // Serial2.println(distance);
  // Serial2.println(analogRead(HALL));
  // delay(200);
  // Linkage::liftBox();
  // delay(2000);
  // Linkage::dropRamp();
  // delay(2000);
  //Serial.println(Sonar::getDist(soundcm));
  //Sonar::detecting(soundcm,90);

  while (stage == 1) {
    servoJoint.write(174);
    Tape_following();

    if(treasure == 0){
      curr_base_pos = Claw::baseRotate(LEFTMOST+2, Claw::base_servo_ptr->read());
      float dist = Sonar::getDist(soundcm);
      if (dist <=20 && dist != 0){
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
    else if(treasure == 1){
      curr_base_pos = Claw::baseRotate(LEFTMOST+2, Claw::base_servo_ptr->read());
      float dist = Sonar::getDist(soundcm);
      if (dist <=20 && dist != 0){
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
    else{
      curr_base_pos = Claw::baseRotate(90, Claw::base_servo_ptr->read());
      Claw::closeClaw();
    }
    
    
  }

  // while(stage == 2) {
  //   IR_following();
  //   Sonar::detecting(soundcm, LEFTMOST);
  // }

  // while(stage == 3) {
    //rampSection();
  // }

}





// void rampSection() {
//   //change numbers
//   int distFromBeacon = 1000;
//   const int countFor90 = 999;
//   const int idealDist = 20;

//   //while further than we want, keep moving
//   while (distFromBeacon>idealDist) {
//     distFromBeacon = Sonar::getDist(soundcm);
//   }

//   encoders1.rightPivotCount(countFor90);
//   encoders1.rightPivotCount(countFor90);
//   encoders1.rightPivotCount(countFor90);

//   Linkage::dropRamp();

//   encoders1.rightPivotCount(countFor90);
//   encoders1.rightPivotCount(countFor90);


//   //rotate left by 180 and drop ramp
//   //rotate left by 180 and move forward
// }


void Tape_following() {
    
    reflectanceL = analogRead(R_L_Sensor);
    reflectanceR = analogRead(R_R_Sensor);
    reflectanceLL = analogRead(R_L_Sensor_2);
    reflectanceRR = analogRead(R_R_Sensor_2);
    Serial.print("reflectance L: "); Serial.print(reflectanceL);
    Serial.print("  reflectance R: "); Serial.print(reflectanceR);
    Serial.print("  reflectance LL: "); Serial.print(reflectanceLL);
    Serial.print("  reflectance RR: "); Serial.print(reflectanceRR);
    Serial.println(" ");
    while(reflectanceL > CW_Threshold && reflectanceR > CW_Threshold) {
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

  while (reflectanceL <= OffTape_Threshold_X && reflectanceR <= OffTape_Threshold_X && reflectanceLL <= Side_Threshold_L && reflectanceRR <= Side_Threshold_R) {
    Tape_follow.tp_motor_offtape();
    delay(50);
    Tape_follow.tp_motor_stop();
    reflectanceL = analogRead(R_L_Sensor);
    reflectanceR = analogRead(R_R_Sensor);
    reflectanceLL = analogRead(R_L_Sensor_2);
    reflectanceRR = analogRead(R_R_Sensor_2);
    // Serial.print("reflectance L: "); Serial.print(reflectanceL);
    // Serial.print("  reflectance R: "); Serial.print(reflectanceR);
    // Serial.print("  reflectance LL: "); Serial.print(reflectanceLL);
    // Serial.print("  reflectance RR: "); Serial.print(reflectanceRR);
    // Serial.println(" ");
    // delay(20);
    int dist_offtape = Sonar::getDist(soundcm);
    if (dist_offtape <= 20 && dist_offtape != 0){
      Tape_follow.tp_motor_stop();
      delay(100);
      Tape::bridge_Right();
      delay(95);
      Tape::tp_motor_stop();
      servoJoint.write(180);
      treasure = Sonar::detecting(soundcm, LEFTMOST, curr_base_pos, dist_offtape);
      Tape_follow.bridge_Back();
      delay(1000);
      reflectanceL = analogRead(R_L_Sensor);
      reflectanceR = analogRead(R_R_Sensor);
      reflectanceLL = analogRead(R_L_Sensor_2);
      reflectanceRR = analogRead(R_R_Sensor_2);
      break;
    }
  
  }

  RL_error = reflectanceL - PID_Threshold_L;
  RR_error = reflectanceR - PID_Threshold_R;

    
  if (RL_error > 0 && RR_error > 0) {
    //**archway//
    error = 0;
    G = 0;
    Tape_follow.tp_motor_straight();
    // Serial.print("straight");
  }
  else if (RL_error < 0 && RR_error > 0) {
    error = RL_error;
    G=Tape_follow.PID(P_value,D_value,error);
    Tape_follow.tp_motor_right(G);
    
    // Serial.print("right");
  }
  else if (RR_error < 0 && RL_error > 0) {
    error = abs(RR_error);
    G=Tape_follow.PID(P_value,D_value,error);
    Tape_follow.tp_motor_left(G);
    
    // Serial.print("left");
  }
  else {
    if (reflectanceRR > Side_Threshold_R) {
      error = -160;
      G=Tape_follow.PID(P_value,D_value,error);
      Tape_follow.tp_motor_right(G);
      // Serial.print("right");
    }
    else if (reflectanceLL > Side_Threshold_L) {
      error = 130;
      G=Tape_follow.PID(P_value,D_value,error);
      Tape_follow.tp_motor_left(G);
      // Serial.print("left");
      if (reflectanceL > Archway_Threshold && reflectanceLL > Archway_Threshold) {
        while (reflectanceRR < Archway_Threshold) {
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
        if (stage == 1) {
          stage++;
        }
      }
    }
  }
  
  Serial.print("G ");
  Serial.println(G);
  Serial.print("error");
  Serial.println(error);

}

void IR_following() {
int i = 0;
int Left_IR;
int Right_IR;
int IR_error;
int G;
int IR_P_value = 9;
int IR_D_value = 2;

  while(i < 2) { //reads left and right IR sensor almost simultaneously
      
    if(i % 2 == 0) {
    read_IR(LEFT_IR);
    Left_IR = analogRead(IR_Sensor);

    } else {
    read_IR(RIGHT_IR);
    Right_IR = analogRead(IR_Sensor);

    }

    i++;

  }

  if(Sonar::getDist(soundcm) < 30) { //checks to see if the robot is close to the end
  IR_error = Left_IR - Right_IR;

    if(IR_error >= -IR_Threshold && IR_error <= IR_Threshold) { //if both sensors are similar to each other
      IR_error = 0;
      G = 0;
      Tape::tp_motor_straight();

    } else if(IR_error > IR_Threshold) {
      G=Tape_follow.PID(IR_P_value,IR_D_value,IR_error);
      Tape::tp_motor_left(G);
  
    } else if (IR_error < -IR_Threshold) {
      IR_error = abs(IR_error);
      G=Tape_follow.PID(IR_P_value,IR_D_value,IR_error);
      Tape::tp_motor_right(G);

    }

  } else {

      if(stage == 2){
      stage++;
    }

  }

}

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