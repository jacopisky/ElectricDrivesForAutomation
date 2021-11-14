#include <arduino-timer.h>

// Arduino pinout for stepper driver
#define A  12
#define Ap 11
#define B  10
#define Bp 9

#define Tstep 2000    // us
#define DELTA_FULL_STEP 1.8
#define DELTA_HALF_STEP 0.9

#define Tcontrol 2000 // us

#define FORWARD   1
#define IDLE_ANG  0
#define BACKWARD -1

const uint8_t half_step[8][4] = {
  {1,0,0,0},
  {1,0,1,0},
  {0,0,1,0},
  {0,1,1,0},
  {0,1,0,0},
  {0,1,0,1},
  {0,0,0,1},
  {1,0,0,1},
};

uint8_t state[4] = {0,0,0,0};
uint8_t i = 0;
int counter = 0;
float target = 0;

Timer<1, micros> timer;

void control(){
  float actual = getStepperAngle();
  if(target > actual){
    stepper(FORWARD);
  }
  else if(target < actual){
    stepper(BACKWARD);
  }
}
  
void setup() {
  pinMode(A,OUTPUT);
  pinMode(Ap,OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(Bp, OUTPUT);

  timer.every(Tcontrol, control);
}


void loop() {
  timer.tick();
}

float getStepperAngle(){
  return DELTA_HALF_STEP * counter;
}

void stepper(uint8_t dir){
  uint8_t prev = i;
  if(dir == FORWARD){
    if(i == 7){
      i = -1;
    }
    i++;
  }
  else if(dir == BACKWARD){
    if(i == 0){
      i = 8;
    }
    i--;
  }
  if(half_step[i][0] != half_step[prev][0]){
    digitalWrite(A, half_step[i][0]);
  }
  if(half_step[i][1] != half_step[prev][1]){
    digitalWrite(Ap, half_step[i][1]);
  }
  if(half_step[i][2] != half_step[prev][2]){
    digitalWrite(B, half_step[i][2]);
  }
  if(half_step[i][3] != half_step[prev][3]){
    digitalWrite(Bp, half_step[i][3]);
  }
}
