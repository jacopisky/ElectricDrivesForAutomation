// ARDUINO PINOUT
// stepper motor driver
#define A  12
#define Ap 11
#define B  10
#define Bp 9
// ultrasonic sensor
#define TRIGGER_PIN_ZERO 4
#define ECHO_PIN_ZERO    2
#define ECHO_PIN_FAR     3
#define TRIGGER_PIN_FAR  5

#define MEASURE_TIMEOUT 2798

// SETUP
#define CALIBRATION_LEN 10

// GENERIC CONSTANTS
#define US_TO_S 0.000001
#define SOUND_SPEED 343.1 // m/s

// CONTROL_INTERVALS
#define T_SENSE 10000    //us
#define T_HALF_STEP 5000 // us

// BALL MOTION CONTROL
// stabilizing feedback control gains
#define Nbar  0.625273120409421
#define K1    -1.054640045447864
#define K2    -1.088045844419463
#define Ki    -0.371802142809193

// low-pass filter for motion measurements
#define FILTER_b0 0.059117397441749
#define FILTER_b1 0.059117397441749
#define FILTER_a0 1
#define FILTER_a1 0.881765205116502

//STEPPER MOTOR CONTROL
// constants for stepper drive
#define DELTA_HALF_STEP 0.9
#define FORWARD   1
#define IDLE_ANG  0
#define BACKWARD -1


const uint8_t half_step[8][4] = {
  {1, 0, 0, 0},
  {1, 0, 1, 0},
  {0, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 0},
  {0, 1, 0, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1},
};

// ultrasonic sensor variables
unsigned long start_t_meas, stop_t_meas;
boolean measure_zero_ready = false, measure_far_ready = false, measure_ready = false;
boolean measuring_zero = false, measuring_far = false, measuring = false;
boolean zero_started = false, far_started = false;
float measure_zero, measure_far;
unsigned long t_prev_meas;
uint8_t j = 0;
boolean calibration_finished = false;
float offset_zero = 0;
float offset_far  = 0;

// filter variables
float ball_pos = 0;
float ball_vel = 0;
float prev_ball_pos = 0;
float prev_ball_vel = 0;
float prev_raw_ball_pos = 0;
float prev_raw_ball_vel = 0;

// stepper variables
uint8_t i = 0;   // index of the stepper motor control's status
int counter = 0; // numer of steps done by the stepper
unsigned long prev_step = 0;

// control variables
float reference = 0.20;
float integrator = 0;
float u = 0;

void change_zero() {
  if (measuring_zero) {
    if (digitalRead(ECHO_PIN_ZERO)) {
      start_t_meas = micros();
      zero_started = true;
    }
    else {
      stop_t_meas = micros();
      measure_zero_ready = true;
    }
  }
}

void startMeasureZero() {
  digitalWrite(TRIGGER_PIN_ZERO, HIGH);
  delayMicroseconds(10);
  measuring_zero = true;
  digitalWrite(TRIGGER_PIN_ZERO, LOW);
}

void change_far() {
  if (measuring_far) {
    if (digitalRead(ECHO_PIN_FAR)) {
      start_t_meas = micros();
      far_started = true;
    }
    else {
      stop_t_meas = micros();
      measure_far_ready = true;
    }
  }
}

void startMeasureFar() {
  digitalWrite(TRIGGER_PIN_FAR, HIGH);
  delayMicroseconds(10);
  measuring_far = true;
  digitalWrite(TRIGGER_PIN_FAR, LOW);
}

void setup() {

  Serial.begin(115200);

  pinMode(A,  OUTPUT);
  pinMode(Ap, OUTPUT);
  pinMode(B,  OUTPUT);
  pinMode(Bp, OUTPUT);

  pinMode(TRIGGER_PIN_FAR,  OUTPUT);
  pinMode(TRIGGER_PIN_ZERO, OUTPUT);
  noInterrupts();
  pinMode(ECHO_PIN_ZERO,    INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_ZERO), change_zero, CHANGE);
  pinMode(ECHO_PIN_FAR,     INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_FAR),  change_far,  CHANGE);
  interrupts();

  for (uint8_t k = 0; k < 45 / DELTA_HALF_STEP + 8; k++) {
    half_stepper(FORWARD);
    delayMicroseconds(T_HALF_STEP);
  }
  counter = 0;
  delay(5000);
}

void loop() {


  if (!measuring) {
    if (micros() - t_prev_meas > T_SENSE) {
      measuring      = true;
    }
    if (measure_ready) {
      measure_ready = false;
      float raw = getPositionTrusted();
      if (!calibration_finished) {
        offset_zero += measure_zero;
        offset_far  += measure_far;
        j++;
        if (j == CALIBRATION_LEN) {
          calibration_finished = true;
          offset_zero = offset_zero / CALIBRATION_LEN;
          offset_far  = offset_far  / CALIBRATION_LEN;
          prev_ball_pos     = raw;
          prev_raw_ball_pos = raw;
        }
      }
      else {
        updateBallDynamics(raw);
        integrator += T_SENSE * US_TO_S * (ball_pos - reference);
        u = (Nbar * reference + K1 * ball_pos + K2 * ball_vel + integrator * Ki) * RAD_TO_DEG;
        Serial.println(ball_pos);
      }
    }
  }
  else {
    if (!measuring_zero && !measuring_far) {
      startMeasureZero();
    }
    if (measuring_zero && zero_started && micros() - start_t_meas >= MEASURE_TIMEOUT) {
      measure_zero_ready = true;
      stop_t_meas = micros();
    }
    if (measuring_zero && measure_zero_ready) {
      measuring_zero = false;
      measure_zero_ready = false;
      zero_started = false;
      float round_time = (float)(stop_t_meas - start_t_meas);
      measure_zero = round_time / 2.0 * US_TO_S * SOUND_SPEED;
      delayMicroseconds(1500);
      startMeasureFar();
    }
    if (measuring_far && far_started && micros() - start_t_meas >= MEASURE_TIMEOUT) {
      measure_far_ready = true;
      stop_t_meas = micros();
    }
    if (measuring_far && measure_far_ready) {
      measuring_far = false;
      far_started = false;
      measuring = false;
      t_prev_meas = micros();
      measure_far_ready = false;
      float round_time = (float)(stop_t_meas - start_t_meas);
      measure_far = round_time / 2.0 * US_TO_S * SOUND_SPEED;
      measure_ready = true;
    }
  }
  
  // STEPPER DRIVE
  if (micros() - prev_step > T_HALF_STEP) {
    float actual = getHalfStepperAngle();
    if (u - actual > DELTA_HALF_STEP) {
      half_stepper(FORWARD);
      prev_step = micros();
    }
    else if (actual - u > DELTA_HALF_STEP) {
      half_stepper(BACKWARD);
      prev_step = micros();
    }
  }
}

void updateBallDynamics(float raw_measure) {
  ball_pos = FILTER_a1 * prev_ball_pos + FILTER_b0 * raw_measure + FILTER_b1 * prev_raw_ball_pos;
  float raw_measure_vel = (ball_pos - prev_ball_pos)*20;
  ball_vel = FILTER_a1 * prev_ball_vel + FILTER_b0 * raw_measure_vel + FILTER_b1 * prev_raw_ball_vel;
  prev_raw_ball_vel = raw_measure_vel;
  prev_ball_vel = ball_vel;
  prev_raw_ball_pos = raw_measure;
  prev_ball_pos = ball_pos;
}

float getPositionTrusted() {
  if (measure_zero > measure_far) {
    return offset_far - measure_far;
  }
  return measure_zero - offset_zero;
}

float getHalfStepperAngle() {
  return DELTA_HALF_STEP * counter;
}

void half_stepper(int dir) {
  uint8_t prev = i;
  if (dir == FORWARD) {
    if (i == 7) {
      i = -1;
    }
    i++;
  }
  else if (dir == BACKWARD) {
    if (i == 0) {
      i = 8;
    }
    i--;
  }
  if (half_step[i][0] != half_step[prev][0]) {
    digitalWrite(A, half_step[i][0]);
  }
  if (half_step[i][1] != half_step[prev][1]) {
    digitalWrite(Ap, half_step[i][1]);
  }
  if (half_step[i][2] != half_step[prev][2]) {
    digitalWrite(B, half_step[i][2]);
  }
  if (half_step[i][3] != half_step[prev][3]) {
    digitalWrite(Bp, half_step[i][3]);
  }
  counter += dir;
}
