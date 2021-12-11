#include <Wire.h>
#include <VL53L0X.h>

#define DATA_LOG

// stabilizing feedback control gains
#define Nbar -0.017248141064664
#define K1    0.0172481410646641
#define K2    0.016428761209089

// low-pass filter
#define FILTER_b0 0
#define FILTER_b1 0.222232320828211
#define FILTER_a0 1
#define FILTER_a1 0.777767679171789

// Arduino pinout
#define A  12
#define Ap 11
#define B  10
#define Bp 9

#define LED_STATUS 13

// constants for stepper drive
#define HALF_Tstep 1100    // us
#define FULL_Tstep 2000    // us

#define DELTA_HALF_STEP 0.9
#define DELTA_FULL_STEP 1.8

#define FORWARD   1
#define IDLE_ANG  0
#define BACKWARD -1

// constants of the control
#define Tcontrol   33000   //us

// distance and speed sensor constants
#define CALIBRATION_LEN 50

// generic constants
#define MM_TO_M 0.001


VL53L0X sensor;


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

uint8_t i = 0;   // index of the stepper motor control's status
int counter = 0; // numer of steps done by the stepper

unsigned long prev_measure_time = 0;
float ball_pos = 0;
float ball_vel = 0;
float prev_ball_pos = 0;
float prev_ball_vel = 0;
float prev_raw_ball_pos = 0;
float prev_raw_ball_vel = 0;

float reference = 0.25;
float u = 0;

float offset = 0;
  
void setup() {
  #ifdef DATA_LOG
  Serial.begin(115200);
  #endif
  pinMode(LED_STATUS, OUTPUT);
  pinMode(A, OUTPUT);
  pinMode(Ap, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(Bp, OUTPUT);
  Wire.begin();
  if (!sensor.init()){
    trap();
  }
  sensor.setMeasurementTimingBudget(Tcontrol);
  sensor.startContinuous();
  uint16_t measures[CALIBRATION_LEN];
  unsigned long accu = 0;
  for(uint8_t i = 0; i < CALIBRATION_LEN; i++){
    while(!sensor.isMeasureReady()){}
    measures[i] = sensor.getMillimeters();
    accu += measures[i];
  }
  offset = accu / ((float)CALIBRATION_LEN) * MM_TO_M;
  /*
  for(uint8_t i = 0; i < CALIBRATION_LEN; i++){
    measures[i] -= (uint16_t)(offset*1000);
  }
  accu = 0;
  for(uint8_t i = 0; i < CALIBRATION_LEN; i++){
    accu += measures[i]* measures[i];
  }
  var = accu / ((float)CALIBRATION_LEN) * MM_TO_M * MM_TO_M;
  */
  while(!sensor.isMeasureReady()){}
  measureBallDynamics();
}

void loop() {
  float actual = getHalfStepperAngle() * DEG_TO_RAD;
  if(u > actual){
    half_stepper(FORWARD);
  }
  else if(u < actual){
    half_stepper(BACKWARD);
  }
  delayMicroseconds(HALF_Tstep);
  if(sensor.isMeasureReady()){
    measureBallDynamics();
    u = Nbar * reference + K1 * ball_pos + K2 * ball_vel;
  }
  #ifdef DATA_LOG
  Serial.print(actual,8);
  Serial.print(F(","));
  Serial.print(prev_ball_pos,8);
  Serial.print(F(","));
  Serial.print(prev_ball_vel,8);
  Serial.print(F(","));
  Serial.print(u);
  Serial.println();
  Serial.flush();
  #endif
}

void measureBallDynamics(){
  uint16_t raw = sensor.getMillimeters();
  unsigned long actual_measure_time = micros();
  float raw_measure = raw * 0.001 - offset;
  ball_pos = FILTER_a1 * prev_ball_pos /*+ FILTER_b0 * raw_measure*/ + FILTER_b1 * prev_raw_ball_pos;
  float raw_measure_vel = (ball_pos - prev_ball_pos) / ((actual_measure_time - prev_measure_time) * 0.000001);
  ball_vel = FILTER_a1 * prev_ball_vel /*+ FILTER_b0 * raw_measure_vel*/ + FILTER_b1 * prev_raw_ball_vel;
  prev_raw_ball_vel = raw_measure_vel;
  prev_ball_vel = ball_vel;
  prev_raw_ball_pos = raw_measure;
  prev_ball_pos = ball_pos;
  prev_measure_time = actual_measure_time;
}

float getHalfStepperAngle(){
  return DELTA_HALF_STEP * counter;
}

void half_stepper(uint8_t dir){
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
