#ifndef CONST_H
#define CONST_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <NewPing.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // This display does not have a reset pin accessible

// all pins defined here

//temporary pot pins
#define P_in PA12 // analog out pin
#define D_in PA15   // analog out pin

//claw
#define stp PB12
#define dir PB13
#define SERVOCLAW PA1
#define SERVOJOINT PA10
#define SERVOBASE PA0

#define HALL PA11

//Reflectance
#define R_L_Sensor PA7
#define R_R_Sensor PA6
#define R_L_Sensor_2 PB0
#define R_R_Sensor_2 PA5
//reflectance edges
#define Left_Sensor PB1
#define Right_Sensor PA4

//IR
#define IR_Sensor PA3
#define IR_Discharge PB5
#define IR_Left_Switch PB10
#define IR_Right_Switch PB11

//Wheels & H-bridge
#define MOTOR_L_F PB_6
#define MOTOR_L_B PB_7
#define MOTOR_R_F PB_8
#define MOTOR_R_B PB_9

//Rotary Enc
#define enc_L PB3
//#define enc_R PB4


//Claw sonar sensor
#define TRIG_PIN PB15
#define ECHO_PIN PB15

//Back sonar sensor
#define BACK_TRIG_PIN PB14
#define BACK_ECHO_PIN PB14

//SONAR CONSTS
#define MAX_DISTANCE 400
#define ITERATIONS 5
#define HUM 40
#define TEMP 20
#define ANGLE 35


//4 bar linkage
#define linkL PA8
#define linkR PA9

///////
//other constants defined here
//////

//clawTreasure
const int CLAWMAX = 150;
const int BASEMAXDISP = 60; //don't change
const int JOINTMAX = 120;
const int LEFTMOST = 90-BASEMAXDISP;
const int RIGHTMOST = 90+BASEMAXDISP;

//rack
const float RETRACTEDPOS = 0;
const float EXTENDEDPOS = 7.5;
const float DEPOSITPOS = 8                                      ;

//HALL
#define bombThreshold 100

//TAPE FOLLOWING
// #define BASE_SPEED 300
// #define RR_Threshold 38
// #define RL_Threshold 84
// #define CW_Threshold 500
// #define RL2_Threshold 100
// #define RR2_Threshold 100

#define MOTOR_FREQ 300

//Encoders
#define PWMFREQ 200
#define FWD_SPEED 300
#define BACK_SPEED 300


// from Zephko:
#define ENC_R_FWD 61
#define ENC_L_FWD 61
#define ENC_STRAIGHT_SPEED 890

#endif