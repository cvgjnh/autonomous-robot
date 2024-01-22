#ifndef TAPEFOLLLOWING_H
#define TAPEFOLLOWING_H


//constants
#define BASE_SPEED 600
#define PID_Threshold_L 200
#define PID_Threshold_R 150
#define Side_Threshold_R 210
#define Side_Threshold_L 300
#define CW_Threshold 900
#define OffTape_Threshold_X 100
#define Archway_Threshold 750


class Tape
{
public:
    Tape(int p_val, int d_val);

    int PID (int P_gain, int D_gain, int R);
    static void tp_motor_left(int gain);
    static void tp_motor_right(int gain);
    static void tp_motor_straight();
    void tp_motor_offtape();
    static void tp_motor_stop();

    void left_turn(int gain);
    void turnR(int leftStop);
    static void bridge_Right();
    static void bridge_Back();
    void handle_interrupt();
    void CW(int speed);

    void tapeSetup();
    void tape_following();

    // VALUES
    volatile uint32_t P_value = 0;
    volatile uint32_t D_value = 0;
    volatile int reflectanceL = 0;
    volatile int reflectanceR = 0;
    volatile int reflectanceLL = 0;
    volatile int reflectanceRR = 0;
    volatile int Left_RFSensor = 0;
    volatile int Right_RFSensor = 0;
    volatile int error = 0;
    volatile int lasterr = 0;
    volatile int interrupt_counter = 0;
    volatile bool abc = true;
};

#endif