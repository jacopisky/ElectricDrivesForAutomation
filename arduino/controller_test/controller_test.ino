// ARDUINO PINOUT
// stepper motor driver
#define A  12
#define Ap 11
#define B  10
#define Bp 9

// ULTRASONIC SENSOR
// arduino pinout
#define TRIGGER_PIN 3
#define ECHO_PIN    2
// setup
#define CALIBRATION_LEN 10
// constants
#define SOUND_SPEED 343.0 // m/s

// generic constants
#define US_TO_S 0.000001

#define T_CONTROL 10000000 //us

// BALL MOTION CONTROL
// stabilizing feedback control gains
#define Nbar  0.467159682754526
#define K1    -2.914478593901547
#define K2    -2.224835373770023
#define Ki    -1.361532353862637
// low-pass filter for motion measurements
#define FILTER_b0 0.111635211704660
#define FILTER_b1 0.111635211704660
#define FILTER_a0 1
#define FILTER_a1 0.776729576590681

//STEPPER MOTOR CONTROL
// constants for stepper drive
#define HALF_Tstep 3000    // us
#define FULL_Tstep 6000    // us

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


// ultrasonic sensor variables
unsigned long start_t_meas, stop_t_meas;
unsigned long prev_meas = 0;
boolean is_meas_ready = false;
float measure;
// calibration global variables
boolean calibration_finished = false;
uint8_t j = 0;
float offset = 0;

// filter variables
float ball_pos = 0;
float ball_vel = 0;
float prev_ball_pos = 0;
float prev_ball_vel = 0;
float prev_raw_ball_pos = 0;
float prev_raw_ball_vel = 0;
float delayed = 0;
unsigned long prev_measure_time;

// STEPPER
uint8_t i = 0;   // index of the stepper motor control's status
int counter = 0; // numer of steps done by the stepper
unsigned long prev_step = 0;

// CONTROL VARIABLES
float reference = 0.25;
float integrator = 0;
float u = 0;


void callback(){
  noInterrupts();
  //byte pinRead = PIND >> ECHO_PIN & B00000001;  // Faster 
  byte pinRead = digitalRead(ECHO_PIN);
  if(pinRead){
    start_t_meas = micros();
  }
  else{
    stop_t_meas = micros(); 
    is_meas_ready = true;
    float round_time = (float)(stop_t_meas - start_t_meas);
    measure = round_time / 2.0 * US_TO_S * SOUND_SPEED;
  }
  interrupts();
}

void startMeasure(){
  noInterrupts();
  if(!is_meas_ready){
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(11);
    digitalWrite(TRIGGER_PIN, LOW);
  }
  interrupts();
}

float readMeasure(){
  noInterrupts();
  is_meas_ready = false;
  prev_meas = micros();
  interrupts();
  return measure;
}
  
void setup() {
  
  Serial.begin(115200);
  
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN,    INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), callback, CHANGE);
  delay(5000);
}

void loop() {
  if(micros() - prev_meas > T_CONTROL){
    startMeasure();
  }
  if(is_meas_ready){
    if(!calibration_finished){
      float tmp = readMeasure();
      offset += tmp;
      j++;
      if(j == CALIBRATION_LEN){
        prev_measure_time = micros();
        prev_raw_ball_pos = tmp;
        calibration_finished = true;
        offset = offset / CALIBRATION_LEN;
      }
    }
    else{
      float tmp = readMeasure();
      if(offset - tmp < 0){tmp = offset;}
      updateBallDynamics(offset-tmp);
      integrator += delayed * (ball_pos - reference);
      u = (Nbar*reference + K1 * ball_pos + K2 * ball_vel + integrator * Ki) * RAD_TO_DEG;
      if(u > 9){
        u = 9;
        integrator -= Ki * delayed * (ball_pos - reference);
      }
      if(u<-9){
        u = -9;
        integrator -= Ki * delayed * (ball_pos - reference);
      }
    }
  }
  if(micros() - prev_step > HALF_Tstep){
    Serial.print(ball_pos);
    Serial.print(F(","));
    Serial.print(ball_vel);
    Serial.print(F(","));
    Serial.println(u);
    float actual = getHalfStepperAngle();
    if(u - actual > DELTA_HALF_STEP){
        half_stepper(FORWARD);
    }
    else if(u - actual < -DELTA_HALF_STEP){
        half_stepper(BACKWARD);
    }
    prev_step = micros();
  }
}

void updateBallDynamics(float raw_measure){
  unsigned long actual_measure_time = micros();
  delayed = (actual_measure_time - prev_measure_time) * US_TO_S;
  ball_pos = FILTER_a1 * prev_ball_pos + FILTER_b0 * raw_measure + FILTER_b1 * prev_raw_ball_pos;
  float raw_measure_vel = (ball_pos - prev_ball_pos);// / delayed;
  ball_vel = FILTER_a1 * prev_ball_vel + FILTER_b0 * raw_measure_vel + FILTER_b1 * prev_raw_ball_vel;
  prev_raw_ball_vel = raw_measure_vel;
  prev_ball_vel = ball_vel;
  prev_raw_ball_pos = raw_measure;
  prev_ball_pos = ball_pos;
  prev_measure_time = actual_measure_time;
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

void full_stepper(uint8_t dir){
  uint8_t prev = i;
  if(dir == FORWARD){
    if(i == 3){
      i = -1;
    }
    i++;
  }
  else if(dir == BACKWARD){
    if(i == 0){
      i = 4;
    }
    i--;
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
