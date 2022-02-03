#include<Math.h>

// ULTRASONIC SENSOR
// arduino pinout
#define TRIGGER_PIN 3
#define ECHO_PIN    2
// setup
#define CALIBRATION_LEN 10
// constants
#define SOUND_SPEED 343.1 // m/s
#define CM_TO_M 0.01
#define US_TO_S 0.000001

// generic constants

#define T_SENSE 1000000 //us
#define T_CONTROL 2000000 //us

// low-pass filter for motion measurements
#define FILTER_b0 0.611015470351657
#define FILTER_b1 0.611015470351657
#define FILTER_a1 -0.222030940703315

#define CALIBRATION_LEN 10

#define TH 0.3

// ultrasonic sensor variables
unsigned long start_t_meas, stop_t_meas;
boolean is_meas_ready = false, measuring = false;
float measure;
unsigned long prev_meas = 0;

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

unsigned long prev_control = 0;

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
  Serial.println(F("position,velocity"));
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN,    INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), change, CHANGE);

}

void loop() {
  if (!measuring && micros() - prev_meas > T_SENSE) {
    startMeasure();
  }
  if (is_meas_ready) {
    is_meas_ready = false;
    prev_meas = micros();
    t_actual_meas = prev_meas;
    measuring = false;
    unsigned long round_time = stop_t_meas - start_t_meas;
    measure = round_time / 2 * SOUND_SPEED * US_TO_S;
    int cm = (int) (measure*100);
    measure = cm * CM_TO_M;
    if (!calibration_finished) {
      offset += measure;
      j++;
      if (j == CALIBRATION_LEN) {
        calibration_finished = true;
        offset = offset / CALIBRATION_LEN;
        if(measure < offset){measure = offset;}
        prev_ball_pos     = offset - measure;
        prev_raw_ball_pos = offset - measure;
        t_prev_meas = t_actual_meas;
      }
    }
    else {
      updateBallDynamics(offset - measure);
      
      if (micros() - prev_control > T_CONTROL) {
          prev_control = micros();
          Serial.println(ball_pos,5);
      }
    }
  }
}

void updateBallDynamics(float raw_measure) {
  //if(raw_measure - prev_raw_ball_pos > TH){return;}
  //if(raw_measure - prev_raw_ball_pos < -TH){return;}
  ball_pos = FILTER_a1 * prev_ball_pos + FILTER_b0 * raw_measure + FILTER_b1 * prev_raw_ball_pos;
  float raw_measure_vel = (ball_pos - prev_ball_pos);///((t_actual_meas - t_prev_meas) * US_TO_S);
  ball_vel = FILTER_a1 * prev_ball_vel + FILTER_b0 * raw_measure_vel + FILTER_b1 * prev_raw_ball_vel;
  prev_raw_ball_vel = raw_measure_vel;
  prev_ball_vel = ball_vel;
  prev_raw_ball_pos = raw_measure;
  prev_ball_pos = ball_pos;
  t_prev_meas = t_actual_meas;
}
