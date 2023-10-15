#ifndef CLAW_C
#define CLAW_C

#include <const.h>
#include <claw.h>

Servo* Claw::claw_servo_ptr;
Servo* Claw::joint_servo_ptr;
Servo* Claw::base_servo_ptr;
float Claw::rackPosition;
int Claw::treasureCount;

void Claw::initializeClaw(Servo* _claw_servo){
    Claw::claw_servo_ptr = _claw_servo;
}

void Claw::initializeJoint(Servo* _joint_servo){
    Claw::joint_servo_ptr = _joint_servo;
}

void Claw::initializeBase(Servo* _base_servo){
    Claw::base_servo_ptr = _base_servo;
}





void Claw::clawSetup() {
  claw_servo_ptr->attach(SERVOCLAW);
  claw_servo_ptr->write(CLAWMAX);
  joint_servo_ptr->attach(SERVOJOINT);
  joint_servo_ptr->write(180);
  base_servo_ptr->attach(SERVOBASE);
  base_servo_ptr->write(90);
  rackPosition = 0;
  treasureCount=0;
  delay(4000);

}

void Claw::rotateZero(int base_current_pos){
    
    if(base_current_pos<90){
        while(base_current_pos<90){
            base_servo_ptr->write(base_current_pos);
            base_current_pos++;
            delay(20);
        }
        
    }
    else if(base_current_pos>90){
        while(base_current_pos>90){
            base_servo_ptr->write(base_current_pos);
            base_current_pos--;
            delay(20);
        }
        
    }
    else{}
}


//returns current position
int Claw::baseRotate(int base_target_pos, int base_current_pos){
    if (base_current_pos< base_target_pos){
        while(base_current_pos < base_target_pos){
            base_servo_ptr->write(base_current_pos);
            base_current_pos++;
            delay(10);
        }
    }
    else if(base_current_pos>base_target_pos){
        while(base_current_pos>base_target_pos){
            base_servo_ptr->write(base_current_pos);
            base_current_pos--;
            delay(10);
        }
    }
    else{}
    return base_current_pos;
}

void Claw::clawJoint(int state){ //only 3 states: raised = 1, lowered = 0, zipline2 = 2, default = 3
    int joint_pos;
    if(state == 1){
        for(joint_pos = 0; joint_pos < JOINTMAX; joint_pos +=1){
            joint_servo_ptr->write(180-joint_pos);
            delay(20);
        }
    }
    else if (state ==0){
        for (joint_pos = JOINTMAX; joint_pos > 0; joint_pos -= 1) { 
            joint_servo_ptr->write(180-joint_pos);        
            delay(10);  
        }  
    }
    else if (state == 3) {
        for(joint_pos = 0; joint_pos < 10; joint_pos +=1){
            joint_servo_ptr->write(180-joint_pos);
            delay(20);
        }
    }
    else{
        for(joint_pos = 0; joint_pos < 90; joint_pos+= 1 ){
            joint_servo_ptr->write(180-joint_pos);
            delay(10);
        }
    }
}

//pinion
void Claw::ForwardStep(float distancecm) 
{
  digitalWrite(dir, HIGH);
  int stepNum = distancecm*340;
  for(int y=0; y<stepNum; y++)
  {
    digitalWrite(stp,HIGH); //Trigger one step
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
  }
  rackPosition += distancecm;
}

void Claw::BackwardStep(float distancecm) 
{
  digitalWrite(dir, LOW);
  int stepNum = distancecm*340;
  for(int y=0; y<stepNum; y++)
  {
    digitalWrite(stp,HIGH); //Trigger one step
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
  }
  rackPosition -= distancecm;
}

//fully retracted is 0 and fully extended is 8.5
void Claw::moveRack(float destinationcm) 
{
    float difference = destinationcm - rackPosition;
    if (difference > 0) {
        ForwardStep(difference);
    }
    else if (difference < 0)
    {
        BackwardStep(-difference);
    }
}

// returns safe = 1 to pick up treasure; if its bomb, claw won't pick up
int Claw::isBomb(){
  int state = digitalRead(HALL);
  int bomb = 0;
  if (state == 0) { //bomb
    bomb = 1;
  }
  else { // treasure
  }
  return bomb;
}

int Claw::closeClaw() {
    int safe = 1;
    for (int claw_pos = CLAWMAX; claw_pos > 0; claw_pos -= 1) { 
        if (isBomb() == 1) {
            safe =0;
        }

        if (safe ==0) {
            break;
        }
        claw_servo_ptr->write(claw_pos);
        delay(10);
    } 
    return safe;

    // bool safe = true;
    // for (int claw_pos = CLAWMAX; claw_pos > 20; claw_pos -= 1) { 
    //     claw_servo_ptr->write(claw_pos);
    //     delay(10);
    // } 
    // safe = !isBomb();
    // if (safe) {
    //     for (int claw_pos = 20; claw_pos > 0; claw_pos -= 1) { 
    //     claw_servo_ptr->write(claw_pos);
    //     delay(10);
    // } 
    // }
    // return safe;
}

void Claw::openClaw() {
    for (int claw_pos = claw_servo_ptr->read(); claw_pos <= CLAWMAX; claw_pos += 1) { 
        claw_servo_ptr->write(claw_pos);
        delay(10);         
    }
}

void Claw::clawPickUp(int current_base_pos){ //sonar successfully detects treasure

    //close claw
    int safe = closeClaw();
    
    if (safe == 1) {
        //centre the base
        //if (treasureCount % 2 == 0) {
            current_base_pos = baseRotate(90, current_base_pos);
        //}
        //else {
            //current_base_pos = baseRotate(90-30,current_base_pos);
        //}
        

        //rack to depositing position
        moveRack(DEPOSITPOS);

        //raise joint
        clawJoint(1);

        //open claw
        openClaw();

        //lower joint
        clawJoint(0);
        
        //return to rack original position
        moveRack(RETRACTEDPOS);

        rotateZero(current_base_pos);

        treasureCount++;
    }
    else {
        openClaw();
        moveRack(RETRACTEDPOS);
        rotateZero(current_base_pos);
    }
}

#endif