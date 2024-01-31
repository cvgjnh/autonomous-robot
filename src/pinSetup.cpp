#include "pinSetup.h"

void pinSetup()
{
    // Wheels
    pinMode(MOTOR_R_F, OUTPUT);
    pinMode(MOTOR_R_B, OUTPUT);
    pinMode(MOTOR_L_F, OUTPUT);
    pinMode(MOTOR_L_B, OUTPUT);

    // tape following
    pinMode(P_in, INPUT_PULLUP);
    pinMode(D_in, INPUT_PULLUP);
    pinMode(R_L_Sensor, INPUT_PULLUP);
    pinMode(R_R_Sensor, INPUT_PULLUP);
    pinMode(R_L_Sensor_2, INPUT_PULLUP);
    pinMode(R_R_Sensor_2, INPUT_PULLUP);
    pinMode(enc_L, INPUT_PULLUP);
    pinMode(Left_Sensor, INPUT_PULLUP);
    pinMode(Right_Sensor, INPUT_PULLUP);

    // claw
    pinMode(stp, OUTPUT);
    pinMode(dir, OUTPUT);

    // hall effect
    pinMode(HALL, INPUT);

    // IR sensor
    pinMode(IR_Left_Switch, OUTPUT);
    pinMode(IR_Right_Switch, OUTPUT);
    pinMode(IR_Sensor, INPUT);
    pinMode(IR_Discharge, OUTPUT);
}