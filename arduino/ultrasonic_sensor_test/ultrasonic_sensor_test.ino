// ULTRASONIC SENSOR
// arduino pinout
#define TRIGGER_PIN 3
#define ECHO_PIN    2
// setup
#define CALIBRATION_LEN 10
// constants
#define SOUND_SPEED 343.1 // m/s

// generic constants
#define US_TO_S 0.000001

#define T_CONTROL 20000 //us

// ultrasonic sensor variables
unsigned long start_t_meas, stop_t_meas;
boolean is_meas_ready = false, measuring = false;
float measure;
unsigned long prev_meas = 0;

void change(){
  if(digitalRead(ECHO_PIN)){
    start_t_meas = micros();
  }
  else{
    stop_t_meas = micros(); 
    is_meas_ready = true;
  }
}

void startMeasure(){
  measuring = true;
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
}


  
void setup() {
  
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN,    INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), change, CHANGE);
    
}

void loop() {
  if(!measuring && micros() - prev_meas > T_CONTROL){
      startMeasure();
    }
    if(is_meas_ready){
      is_meas_ready = false;
      prev_meas = micros();
      measuring = false;
      float round_time = (float)(stop_t_meas - start_t_meas);
      measure = round_time / 2.0 * US_TO_S * SOUND_SPEED;
      Serial.println(measure,10);
    }
}
