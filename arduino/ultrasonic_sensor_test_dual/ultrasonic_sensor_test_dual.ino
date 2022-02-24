// ULTRASONIC SENSOR
// arduino pinout
#define TRIGGER_PIN_ZERO 4
#define ECHO_PIN_ZERO    3
#define ECHO_PIN_FAR     2
#define TRIGGER_PIN_FAR  5
// setup
#define CALIBRATION_LEN 10
// constants
#define SOUND_SPEED 343.1 // m/s

// generic constants
#define US_TO_S 0.000001

#define T_CONTROL 1000000 //us

// ultrasonic sensor variables
unsigned long start_t_meas, stop_t_meas;
boolean measure_zero_ready = false, measure_far_ready = false, measure_ready = false;
boolean measuring_zero = false, measuring_far = false, measuring = false;
float measure_zero, measure_far;
unsigned long prev_meas = 0;

void change_zero() {
  if (measuring_zero) {
    if (digitalRead(ECHO_PIN_ZERO)) {
      start_t_meas = micros();
      Serial.println(F("zero start"));
    }
    else {
      stop_t_meas = micros();
      Serial.println(F("zero stop"));
      measure_zero_ready = true;
    }
  }
}

void startMeasureZero() {
  Serial.println(F("init zero"));
  digitalWrite(TRIGGER_PIN_ZERO, HIGH);
  delayMicroseconds(10);
  measuring_zero = true;
  digitalWrite(TRIGGER_PIN_ZERO, LOW);
}

void change_far() {
  if (measuring_far) {
    if (digitalRead(ECHO_PIN_FAR)) {
      start_t_meas = micros();
      Serial.println(F("far start"));
    }
    else {
      stop_t_meas = micros();
      Serial.println(F("far stop"));
      measure_far_ready = true;
    }
  }
}

void startMeasureFar() {
  Serial.println(F("init far"));
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
      measuring = true;
    }
    if(measure_ready){
      measure_ready = false;
      Serial.println(measure_zero, 5);
      Serial.println(measure_far, 5);    
    }
  }
  else {
    if (!measuring_zero && !measuring_far) {
      startMeasureZero();
    }
    if (measuring_zero && measure_zero_ready) {
      measure_zero_ready = false;
      measuring_zero = false;
      float round_time = (float)(stop_t_meas - start_t_meas);
      measure_zero = round_time / 2.0 * US_TO_S * SOUND_SPEED;
      delay(1);
      startMeasureFar();
    }
    if (measuring_far && measure_far_ready) {
      measuring = false;
      measuring_far = false;
      prev_meas = micros();
      measure_far_ready = false;
      float round_time = (float)(stop_t_meas - start_t_meas);
      measure_far = round_time / 2.0 * US_TO_S * SOUND_SPEED;
      measure_ready = true;
    }
  }
}
