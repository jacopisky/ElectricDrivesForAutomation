// ARDUINO PINOUT
// stepper motor driver
#define A  12
#define Ap 11
#define B  10
#define Bp 9
// ultrasonic sensor
#define TRIGGER_PIN 3
#define ECHO_PIN    2

// SETUP
#define CALIBRATION_LEN 10

// GENERIC CONSTANTS
#define US_TO_S 0.000001
#define SOUND_SPEED 343.1 // m/s

// CONTROL_INTERVALS
#define T_SENSE 10000   //us
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
boolean is_meas_ready = false, measuring = false;
float measure;

uint8_t j = 0;
boolean calibration_finished = false;
float offset = 0;

// filter variables
float ball_pos = 0;
float ball_vel = 0;
float prev_ball_pos = 0;
float prev_ball_vel = 0;
float prev_raw_ball_pos = 0;
float prev_raw_ball_vel = 0;
unsigned long t_actual_meas, t_prev_meas;

// STEPPER
uint8_t i = 0;   // index of the stepper motor control's status
int counter = 0; // numer of steps done by the stepper
unsigned long prev_step = 0;

// CONTROL VARIABLES
float reference = 0.20;
float integrator = 0;
float u = 0;


void change() {
  if (digitalRead(ECHO_PIN)) {
    start_t_meas = micros();
  }
  else {
    stop_t_meas = micros();
    is_meas_ready = true;
  }
}

void startMeasure() {
  measuring = true;
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
}

void setup() {

  Serial.begin(115200);
  delay(1000);
  pinMode(A, OUTPUT);
  pinMode(Ap, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(Bp, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN,    INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), change, CHANGE);

  for (uint8_t k = 0; k < 45 / 9 * 10 + 9; k++) {
    half_stepper(FORWARD);
    delayMicroseconds(T_HALF_STEP);
  }
  counter = 0;
  delay(5000);

}

void loop() {
  if (!measuring && micros() - t_prev_meas > T_SENSE) {
    startMeasure();
  }
  if (is_meas_ready) {
    is_meas_ready = false;
    t_actual_meas = micros();
    measuring = false;
    unsigned long round_time = stop_t_meas - start_t_meas;
    measure = round_time / 2 * SOUND_SPEED * US_TO_S;
    if (!calibration_finished) {
      offset += measure;
      j++;
      if (j == CALIBRATION_LEN) {
        calibration_finished = true;
        offset = offset / CALIBRATION_LEN;
        prev_ball_pos     = offset - measure;
        prev_raw_ball_pos = offset - measure;
      }
    }
    else {
      float raw = offset - measure;
      integrator += (t_actual_meas - t_prev_meas) * US_TO_S * (ball_pos - reference);
      u = (Nbar * reference + K1 * ball_pos + K2 * ball_vel + integrator * Ki) * RAD_TO_DEG;
      updateBallDynamics(raw);
      Serial.println(ball_pos, 5);
    }

    t_prev_meas = t_actual_meas;
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
  float raw_measure_vel = ball_pos - prev_ball_pos;
  ball_vel = FILTER_a1 * prev_ball_vel + FILTER_b0 * raw_measure_vel + FILTER_b1 * prev_raw_ball_vel;
  prev_raw_ball_vel = raw_measure_vel;
  prev_ball_vel = ball_vel;
  prev_raw_ball_pos = raw_measure;
  prev_ball_pos = ball_pos;
  t_prev_meas = t_actual_meas;
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
