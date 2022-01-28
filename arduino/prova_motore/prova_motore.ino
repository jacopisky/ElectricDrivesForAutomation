// Arduino pinout for stepper driver
#define A  12
#define Ap 11
#define B  10
#define Bp 9

#define HALF_Tstep 1000    // us
#define FULL_Tstep 2000    // us

#define DELTA_HALF_STEP 0.9
#define DELTA_FULL_STEP 1.8

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

const uint8_t full_step[4][4] = {
  {1,0,0,0},
  {0,0,1,0},
  {0,1,0,0},
  {0,0,0,1},
};

uint8_t i = 0;
int counter = 0;
float target = 360;
  
void setup() {
  pinMode(A,OUTPUT);
  pinMode(Ap,OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(Bp, OUTPUT);
}


void loop() {
  float actual = getHalfStepperAngle();
  if(target > actual){
    half_stepper(FORWARD);
  }
  else if(target < actual){
    half_stepper(BACKWARD);
  }
  delayMicroseconds(HALF_Tstep);
}

float getHalfStepperAngle(){
  return DELTA_HALF_STEP * counter;
}

float getFullStepperAngle(){
  return DELTA_FULL_STEP * counter;
}

void half_stepper(int dir){
  uint8_t prev = i;
  if(dir == FORWARD){
    if(i == 7){
      i = 0;
    }
    else{
      i++;
    }
  }
  else if(dir == BACKWARD){
    if(i == 0){
      i = 7;
    }
    else{
      i--;
    }
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
  counter += dir;
}

void full_stepper(int dir){
  uint8_t prev = i;
  if(dir == FORWARD){
    if(i == 3){
      i = 0;
    }
    else{
      i++;
    }
  }
  else if(dir == BACKWARD){
    if(i == 0){
      i = 3;
    }
    else{
      i--;
    }
  }
  if(full_step[i][0] != full_step[prev][0]){
    digitalWrite(A, full_step[i][0]);
  }
  if(full_step[i][1] != full_step[prev][1]){
    digitalWrite(Ap, full_step[i][1]);
  }
  if(full_step[i][2] != full_step[prev][2]){
    digitalWrite(B, full_step[i][2]);
  }
  if(full_step[i][3] != full_step[prev][3]){
    digitalWrite(Bp, full_step[i][3]);
  }
  counter += dir;
}
