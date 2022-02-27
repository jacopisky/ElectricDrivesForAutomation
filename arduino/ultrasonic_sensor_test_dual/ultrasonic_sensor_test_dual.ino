// ULTRASONIC SENSOR
// arduino pinout
#define TRIGGER_PIN_ZERO 4
#define ECHO_PIN_ZERO    2
#define ECHO_PIN_FAR     3
#define TRIGGER_PIN_FAR  5
// constants
#define SOUND_SPEED 343.1 // m/s

// generic constants
#define US_TO_S 0.000001

#define T_CONTROL 1000000 //us

#define MEASURE_TIMEOUT 2798

// ultrasonic sensor variables
unsigned long start_t_meas, stop_t_meas;
boolean measure_zero_ready = false, measure_far_ready = false, measure_ready = false;
boolean zero_started = false, far_started = false;
boolean measuring_zero = false, measuring_far = false, measuring = false;
float measure_zero, measure_far;
unsigned long prev_meas = 0;

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
  pinMode(TRIGGER_PIN_FAR,   OUTPUT);
  pinMode(TRIGGER_PIN_ZERO,   OUTPUT);
  noInterrupts();
  pinMode(ECHO_PIN_ZERO, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_ZERO), change_zero, CHANGE);
  pinMode(ECHO_PIN_FAR,  INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_FAR),  change_far,  CHANGE);
  interrupts();
}

void loop() {


  if (!measuring) {
    if (micros() - prev_meas > T_CONTROL) {
      measuring      = true;
    }
    if (measure_ready) {
      measure_ready = false;
      Serial.print(measure_zero);
      Serial.print(F("   "));
      Serial.println(measure_far);
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
      prev_meas = micros();
      measure_far_ready = false;
      float round_time = (float)(stop_t_meas - start_t_meas);
      measure_far = round_time / 2.0 * US_TO_S * SOUND_SPEED;
      measure_ready = true;
    }
  }
}
