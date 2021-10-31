#define T 2000  // 1700 vibbra ma ok
void setup() {
  // put your setup code here, to run once:
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
}
int i = 0;
void loop() {
  while(i<12){
  // put your main code here, to run repeatedly:
  digitalWrite(12,HIGH);
  delayMicroseconds(T);
  digitalWrite(12,LOW);
  digitalWrite(10,HIGH);
  delayMicroseconds(T);
  digitalWrite(10,LOW);
  digitalWrite(11,HIGH);
  delayMicroseconds(T);
  digitalWrite(11,LOW);
  digitalWrite(9,HIGH);
  delayMicroseconds(T);
  digitalWrite(9,LOW);
  i++;
  }
  delay(2000);
  i = 0;
}
