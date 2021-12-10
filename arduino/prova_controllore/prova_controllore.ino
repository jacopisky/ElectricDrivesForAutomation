#include <arduino-timer.h>
#include <Wire.h>
#include <VL53L0X.h>

// control gains
#define Nbar -0.017248141064664
#define K1    0.017248141064664
#define K2    0.016428761209089

// Arduino pinout for stepper driver
#define A  12
#define Ap 11
#define B  10
#define Bp 9

#define LED_STATUS 13

#define DELTA_HALF_STEP 0.9

#define Tcontrol 100000 // us
#define Tstep    80000  // us

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
Timer<2, micros> timer2;

VL53L0X sensor;

float prev_ball_pos = 0;
unsigned long prev_time = 0;

float u = 0;

void control(){
  Serial.println(F("c"));
  if(sensor.isMeasureReady()){
    float ball_pos = sensor.getMillimeters() * 0.001;
    unsigned long actual_time = micros();
    float ball_velocity = (ball_pos - prev_ball_pos) / (actual_time - prev_time);
    u = Nbar * target + K1 * ball_pos + K2 * ball_velocity;
    prev_ball_pos = ball_pos;
    prev_time = actual_time;
  }
}

void stepper_control(){
  
  Serial.println(F("s"));
  float actual_motor_angle = getStepperAngle() * DEG_TO_RAD;
  if(u > actual_motor_angle){
    stepper(FORWARD);
  }
  else if(u < actual_motor_angle){
    stepper(BACKWARD);
  }
}
  
void setup() {
  Serial.begin(115200);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(A, OUTPUT);
  pinMode(Ap, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(Bp, OUTPUT);

  Wire.begin();

  sensor.setTimeout(Tcontrol/1000);
  if (!sensor.init()){
    trap();
  }
  sensor.startContinuous(Tcontrol/1000);
  while(!sensor.isMeasureReady()){}
  
  prev_ball_pos = sensor.getMillimeters();
  prev_time = micros();
  timer.every(Tstep, stepper_control);
  //timer2.every(Tcontrol, control);
  
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
  counter += dir;
}

void trap(){
  while(1){
    digitalWrite(LED_STATUS, HIGH);
    delay(1000);
    digitalWrite(LED_STATUS, LOW);
    delay(1000);
  }  
}
