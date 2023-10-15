#include "SensorArray.h"
#include "const.h"

class Encoders
{
public:
    Encoders();
    void handle_R_interrupt();
    void handle_L_interrupt();
    void drive(int leftStop,int speedL);
    void turnL(int leftStop);
    void turnR(int leftStop);
    void backup(int leftStop, int speedL);
    void stop();
    void rightPivot();
    void rightPivotCount(int counts);
    void adjustmentBackup();
    void adjustmentBackupCount(int counts);
    volatile int countL;
    volatile int countR;
};