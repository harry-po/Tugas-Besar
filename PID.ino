/******Motor 1*****/
#define PWM1   PB1
#define ENM1   PC5
#define MENCA1 PD4
#define MENCB1 PD3

/*****Motor 2******/
#define PWM2    PE14
#define ENM2    PE11
#define MENCA2  PD2
#define MENCB2  PD1

/******Motor 3*****/
#define PWM3   PA6
#define ENM3   PA5
#define MENCA3 PD0
#define MENCB3 PC12

/******Motor 4*****/
#define PWM4   PB10
#define ENM4   PE15
#define MENCA4 PC11
#define MENCB4 PC10

/*********RPM Variabel*************/
unsigned long timeold;
float counter1, counter2, counter3, counter4;
float rotasi1, rotasi2, rotasi3, rotasi4;
float ppr = 275.0;
float rpm1, rpm2, rpm3, rpm4;
/*********Regresion Variabel*******/
float RPM1, RPM2, RPM3, RPM4;
/*********INPUT Parameter**********/
int x = 50;

/*******PID Variabel**********/
float kp1 = 10; //motor 1
float kp2 = 1;
float kp3 = 1; //1.2
float kp4 = 1; //0.98

float ki1 = 0;
float ki2 = 0;
float ki3 = 0;
float ki4 = 0;

float kd1 = 0;
float kd2 = 0;
float kd3 = 0;
float kd4 = 0;

//float ki1 = 0.004;
//float ki2 = 0.004;
//float ki3 = 0.004;
//float ki4 = 0.004;
//
//float kd1 = 0.03;
//float kd2 = 0.03;
//float kd3 = 0.03;
//float kd4 = 0.03;
/********Local Control************/
float PID1, PID2, PID3, PID4;
float eror1, eror2,eror3,eror4;

float sumeror1 = 0;
float sumeror2 = 0;
float sumeror3 = 0;
float sumeror4 = 0;
float prev1 = 0;
float prev2 = 0;
float prev3 = 0;
float prev4 = 0;

void setup() {
  // put your setup code here, to run once:
  HardwareSerial Serial1(PA2, PA3);
  Serial.begin(115200);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);

  pinMode(ENM1, OUTPUT);
  pinMode(ENM2, OUTPUT);
  pinMode(ENM3, OUTPUT);
  pinMode(ENM4, OUTPUT);

  pinMode(MENCA1, INPUT);
  pinMode(MENCB1, INPUT);

  pinMode(MENCA2, INPUT);
  pinMode(MENCB2, INPUT);

  pinMode(MENCA3, INPUT);
  pinMode(MENCB3, INPUT);

  pinMode(MENCA4, INPUT);
  pinMode(MENCB4, INPUT);

  attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);
}

void loop() {
  // put your main code here, to run repeatedl1y:
  RPM1 = 0.8646 * x - 9.9958;
  RPM2 = 0.8563 * x - 3.4796; //Target
  RPM3 = 0.8985 * x - 3.0178;
  RPM4 = 0.841 * x - 12.81;

  eror1 = RPM1 - rpm1;
  eror2 = RPM2 - rpm2;
  eror3 = RPM3 - rpm3;
  eror4 = RPM4 - rpm4;

  PID1 = (kp1 * eror1) + (ki1 * sumeror1) + (kd1 * (eror1 - prev1));
  prev1 = eror1;
  sumeror1 += eror1;

  PID2 = (kp2 * eror2) + (ki2 * sumeror2) + (kd2 * (eror2 - prev2));
  prev2 = eror2;
  sumeror2 += eror2;

  PID3 = (kp3 * eror3) + (ki3 * sumeror3) + (kd3 * (eror3 - prev3));
  prev3 = eror3;
  sumeror3 += eror3;

  PID4 = (kp4 * eror4) + (ki4 * sumeror4) + (kd4 * (eror4 - prev4));
  prev4 = eror4;
  sumeror4 += eror4;
   if (ppr <= counter ){
    PID1 = 0;
   }
  analogWrite(PWM1, PID1); //motor1
  analogWrite(PWM2, PID2); //motor2
  analogWrite(PWM3, PID3); //motor3
  analogWrite(PWM4, PID4); //motor4

  digitalWrite(ENM1, HIGH);
//  digitalWrite(ENM2, HIGH);
//  digitalWrite(ENM3, HIGH);
//  digitalWrite(ENM4, HIGH);
  delay(1000);

  detachInterrupt(MENCA1);
  detachInterrupt(MENCA2);
  detachInterrupt(MENCA3);
  detachInterrupt(MENCA4);
  detachInterrupt(MENCA4);

  rotasi1 = counter1 / ppr;
  rotasi2 = counter2 / ppr;
  rotasi3 = counter3 / ppr;
  rotasi4 = counter4 / ppr;

  counter1 = 0;
  counter2 = 0;
  counter3 = 0;
  counter4 = 0;

  rpm1 = 60.0 * 1000.0 / (millis() - timeold) * rotasi1;
  rpm2 =  60.0 * 1000.0 / (millis() - timeold) * rotasi2;
  rpm3 =  60.0 * 1000.0 / (millis() - timeold) * rotasi3;
  rpm4 =  60.0 * 1000.0 / (millis() - timeold) * rotasi4;
  timeold = millis();

  Serial.print(RPM2);
  Serial.print(" ");
  Serial.println(rpm1);
//  Serial.print(" ");
//  Serial.print(rpm2);
//  Serial.print(" ");
//  Serial.print(rpm3);
//  Serial.print(" ");
//  Serial.println(rpm4);

  attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);
}


void sumcounter1() {
  if (digitalRead(MENCB1)) {
    counter1 += 1;
  }
  else {
    counter1 -= 1;
  }
}

void sumcounter2() {
  if (digitalRead(MENCB2)) {
    counter2 += 1;
  }
  else {
    counter2 -= 1;
  }
}

void sumcounter3() {
  if (digitalRead(MENCB3)) {
    counter3 += 1;
  }
  else {
    counter3 -= 1;
  }
}

void sumcounter4() {
  if (digitalRead(MENCB4)) {
    counter4 += 1;
  }
  else {
    counter4 -= 1;
  }
}
