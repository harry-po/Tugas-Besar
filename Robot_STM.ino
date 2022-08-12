#include <SerialCommands.h>
#include <Servo.h>

/**LED Internal***/
#define MAJU   PD13
#define KIRI   PD12
#define KANAN  PD14
#define MUNDUR PD15

/******Motor 1*****/
#define PWM12  PB0 //pin_pwm_2 
#define PWM11  PB1
#define ENM1   PC5 //pin_enable
#define MENCA1 PD4
#define MENCB1 PD3

/*****Motor 2******/
#define PWM21   PE14 //pin_pwm2_1
#define PWM22   PE13
#define ENM2    PE11 //pin_enablem3
#define MENCA2  PD2
#define MENCB2  PD1

/******Motor 3*****/
#define PWM31  PA6 //pin_pwm3_1 
#define PWM32  PA7
#define ENM3   PA5 //pin_enablem2
#define MENCA3 PD0
#define MENCB3 PC12

/******Motor 4*****/
#define PWM41  PB10 //pin_pwm4_2
#define PWM42  PB11
#define ENM4   PE15 //pin_enablem4
#define MENCA4 PC11
#define MENCB4 PC10

/******Encoder Internal Motor*****/
#define MENCA1 PD4
#define MENCB1 PD3
#define MENCA2 PD2
#define MENCB2 PD1
#define MENCA3 PD0
#define MENCB3 PC12
#define MENCA4 PC11
#define MENCB4 PC10

/********PIN ECHO*********/
#define ECHO1 PB7
#define ECHO2 PB5
#define ECHO3 PB3
#define ECHO4 PD6

/********PIN TRIGGER *******/
#define TRIGGER1 PB6
#define TRIGGER2 PB4
#define TRIGGER3 PD7
#define TRIGGER4 PD5

/*********PIN Buzzer*******/
#define BUZZER PD9

float distance = 0;
float distance1 = 0;
float distance2 = 0;
float distance3 = 0;
float sound = 0.034 / 2;
int duration, duration1, duration2, duration3;
float calibration;

/*****pwm input******/
float pwm = 30.0;
float pwm1 = 30.0;
float pwm2  = 30.0;
float pwm3 = 30.0;

/*********RPM Variabel*************/
unsigned long timeold;
float counter1, counter2, counter3, counter4;
float rotasi1, rotasi2, rotasi3, rotasi4;
float ppr = 275.0;
float rpm1, rpm2, rpm3, rpm4;
/*********Regresion Variabel*******/
float RPM1, RPM2, RPM3, RPM4;
/*********INPUT Parameter/**********/
int x = 50;
int y = 0;
int z = 0;

/*******PID Variabel**********/
float kp1 = 1.4; //motor 1
float kp2 = 1.3;
float kp3 = 1.2; //1.2
float kp4 = 0.95; //0.98

float ki1 = 0.004;
float ki2 = 0.004;
float ki3 = 0.004;
float ki4 = 0.004;

float kd1 = 0.03;
float kd2 = 0.03;
float kd3 = 0.03;
float kd4 = 0.03;

/********Local Control************/
float PID1, PID2, PID3, PID4;
float eror1, eror2, eror3, eror4;
float sumeror1 = 0;
float sumeror2 = 0;
float sumeror3 = 0;
float sumeror4 = 0;
float prev1 = 0;
float prev2 = 0;
float prev3 = 0;
float prev4 = 0;

/*****Serial Communication******/
char ReceiveBuffer[32];
SerialCommands serial_commands(&Serial, ReceiveBuffer, sizeof(ReceiveBuffer), "?", ":");

/*****konfigurasi Analog Kiri******/
Servo vServo; //Vertikal
Servo hServo; //Horizontal
int vAngle = 90;
int hAngle = 90;
int vSpeed = 0;
int hSpeed = 0;
int updateTime = 0;

/*****Konfigurasi Serial Komunikasi******/
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("[");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

/*****Analog Kiri******/
void set_speed(SerialCommands* sender)  {
  int motor1Spd = atoi(sender->Next()); // 0-255
  int motor1Dir = atoi(sender->Next()); // 0: backwards, 1: forward (kiri)
  int motor2Spd = atoi(sender->Next()); // 0-255
  int motor2Dir = atoi(sender->Next()); // 0: backwards, 1: forward (kanan)

  if (motor1Dir == 1) {

    digitalWrite(MAJU, HIGH); //area kanan maju
    analogWrite(PWM31, motor1Spd);
    analogWrite(PWM41, motor1Spd);
    digitalWrite(ENM3, HIGH);
    digitalWrite(ENM4, HIGH);
  } else if (motor1Dir == 0) {

    digitalWrite(MUNDUR, HIGH); //area kanan mundur
    analogWrite(PWM11, motor1Spd);
    analogWrite(PWM22, motor1Spd);
    digitalWrite(ENM1, HIGH);
    digitalWrite(ENM2, HIGH);
  }
  if (motor2Dir == 1) {

    digitalWrite(KIRI, HIGH); //area kiri maju

    analogWrite(PWM21, motor2Spd); //motor3
    analogWrite(PWM12, motor2Spd); //motor4
    //
    digitalWrite(ENM1, HIGH);
    digitalWrite(ENM2, HIGH);


  } else if (motor2Dir == 0) {

    digitalWrite(KANAN, HIGH); //area kiri mundur
    analogWrite(PWM32, motor2Spd);
    analogWrite(PWM42, motor2Spd);
    digitalWrite(ENM3, HIGH);
    digitalWrite(ENM4, HIGH);
  }
  else {
    digitalWrite(MAJU, LOW);
    digitalWrite(MUNDUR, LOW);
    digitalWrite(KIRI, LOW);
    digitalWrite(KANAN, LOW);
    Stop();
  }
}

/*****Analog Kanan******/
void move_servo(SerialCommands* sender) {
  int spd = atoi(sender->Next()); // 0-10
  int dir = atoi(sender->Next()); // 0: stop, 1:DEPAN, 2:KANAN, 3:BELAKANG, 4:KIRI

  // Servo speed adjustment based on the command.
  if (dir == 1) {
    Reset_Val();
    digitalWrite(MAJU, HIGH); //DEPAN
    RPM1 = 0.8646 * 50 - 9.9958;
    RPM2 = 0.8563 * 50 - 3.4796; //Target
    RPM3 = 0.8985 * 50 - 3.0178;
    RPM4 = 0.841 * 50 - 12.81;

    eror1 = RPM2 - rpm1;
    eror2 = RPM2 - rpm2;
    eror3 = RPM2 - rpm3;
    eror4 = RPM2 - rpm4;

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

    analogWrite(PWM12, PID1); //motor1
    analogWrite(PWM11, 0);
    analogWrite(PWM21, PID2); //motor2
    analogWrite(PWM22, 0); //motor1
    analogWrite(PWM31, PID3); //motor3
    analogWrite(PWM32, 0); //motor1
    analogWrite(PWM41, PID4); //motor4
    analogWrite(PWM42, 0); //motor4
    //
    digitalWrite(ENM1, HIGH);
    digitalWrite(ENM2, HIGH);
    digitalWrite(ENM3, HIGH);
    digitalWrite(ENM4, HIGH);
    delay(10);

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
    delay(50);
    attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);
  } else if (dir == 2) {
    Reset_Val();
    digitalWrite(MAJU, HIGH); //DEPAN
    RPM1 = 0.8646 * 50 - 9.9958;
    RPM2 = 0.8563 * 50 - 3.4796; //Target
    RPM3 = 0.8985 * 50 - 3.0178;
    RPM4 = 0.841 * 50 - 12.81;

    eror1 = RPM2 - rpm1;
    eror2 = RPM2 - rpm2;
    eror3 = RPM2 - rpm3;
    eror4 = RPM2 - rpm4;

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

    analogWrite(PWM12, 0); //motor1
    analogWrite(PWM11, PID1);
    analogWrite(PWM21, PID2); //motor2
    analogWrite(PWM22, 0); //motor1
    analogWrite(PWM31, 0); //motor3
    analogWrite(PWM32, PID3); //motor1
    analogWrite(PWM41, PID4); //motor4
    analogWrite(PWM42, 0); //motor4
    //
    digitalWrite(ENM1, HIGH);
    digitalWrite(ENM2, HIGH);
    digitalWrite(ENM3, HIGH);
    digitalWrite(ENM4, HIGH);
    delay(10);

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
    delay(50);
    attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);
  } else if (dir == 3) {
    Reset_Val();
    RPM1 = 0.8646 * x - 9.9958;
    RPM2 = 0.8563 * x - 3.4796; //Target
    RPM3 = 0.8985 * x - 3.0178;
    RPM4 = 0.841 * x - 12.81;

    eror1 = RPM2 - rpm1;
    eror2 = RPM2 - rpm2;
    eror3 = RPM2 - rpm3;
    eror4 = RPM2 - rpm4;

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

    analogWrite(PWM12, 0); //motor1
    analogWrite(PWM11, PID1);
    analogWrite(PWM21, 0); //motor2
    analogWrite(PWM22, PID2); //motor1
    analogWrite(PWM31, 0); //motor3
    analogWrite(PWM32, PID3); //motor1
    analogWrite(PWM41, 0); //motor4
    analogWrite(PWM42, PID4); //motor4
    //
    digitalWrite(ENM1, HIGH);
    digitalWrite(ENM2, HIGH);
    digitalWrite(ENM3, HIGH);
    digitalWrite(ENM4, HIGH);
    digitalWrite(MUNDUR, HIGH); //BELAKANG
    delay(10);

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

    delay(50);
    attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);

  } else if (dir == 4) {
    Reset_Val();
    digitalWrite(MAJU, HIGH); //DEPAN
    RPM1 = 0.8646 * 50 - 9.9958;
    RPM2 = 0.8563 * 50 - 3.4796; //Target
    RPM3 = 0.8985 * 50 - 3.0178;
    RPM4 = 0.841 * 50 - 12.81;

    eror1 = RPM2 - rpm1;
    eror2 = RPM2 - rpm2;
    eror3 = RPM2 - rpm3;
    eror4 = RPM2 - rpm4;

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

    analogWrite(PWM12, PID1); //motor1
    analogWrite(PWM11, 0);
    analogWrite(PWM21, 0); //motor2
    analogWrite(PWM22, PID2); //motor1
    analogWrite(PWM31, PID3); //motor3
    analogWrite(PWM32, 0); //motor1
    analogWrite(PWM41, 0); //motor4
    analogWrite(PWM42, PID4); //motor4
    //
    digitalWrite(ENM1, HIGH);
    digitalWrite(ENM2, HIGH);
    digitalWrite(ENM3, HIGH);
    digitalWrite(ENM4, HIGH);
    delay(10);

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
    delay(50);
    attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);

  } else {
    digitalWrite(MAJU, LOW);
    digitalWrite(MUNDUR, LOW);
    digitalWrite(KIRI, LOW);
    digitalWrite(KANAN, LOW);

    Stop();
  }
}

/*****Tombol kanan******/
void right_buttons(SerialCommands* sender) {
  int buttonIndex = atoi(sender->Next()); // 1: A, 2: B, 3: X, 4: Y
  // tombol x,y,a,b
  if (buttonIndex == 4) {
    Reset_Val();
    digitalWrite(MAJU, HIGH); //DEPAN
    RPM1 = 0.8646 * 50 - 9.9958;
    RPM2 = 0.8563 * 50 - 3.4796; //Target
    RPM3 = 0.8985 * 50 - 3.0178;
    RPM4 = 0.841 * 50 - 12.81;

    eror1 = RPM2 - rpm1;
    eror2 = RPM2 - rpm2;
    eror3 = RPM2 - rpm3;
    eror4 = RPM2 - rpm4;

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

    analogWrite(PWM12, PID1); //motor1
    analogWrite(PWM11, 0);
    analogWrite(PWM21, PID2); //motor2
    analogWrite(PWM22, 0); //motor1
    analogWrite(PWM31, PID3); //motor3
    analogWrite(PWM32, 0); //motor1
    analogWrite(PWM41, PID4); //motor4
    analogWrite(PWM42, 0); //motor4
    //
    digitalWrite(ENM1, HIGH);
    digitalWrite(ENM2, HIGH);
    digitalWrite(ENM3, HIGH);
    digitalWrite(ENM4, HIGH);
    delay(10);

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
    delay(50);
    attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);
  } else if (buttonIndex == 1) {
    Reset_Val();
    digitalWrite(KANAN, HIGH); //DEPAN
    RPM1 = 0.8646 * 30 - 9.9958;
    RPM2 = 0.8563 * 30 - 3.4796; //Target
    RPM3 = 0.8985 * 30 - 3.0178;
    RPM4 = 0.841 * 30 - 12.81;

    eror1 = RPM2 - rpm1;
    eror2 = RPM2 - rpm2;
    eror3 = RPM2 - rpm3;
    eror4 = RPM2 - rpm4;

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

    analogWrite(PWM12, 0); //motor1
    analogWrite(PWM11, PID1);
    analogWrite(PWM21, PID2); //motor2
    analogWrite(PWM22, 0); //motor1
    analogWrite(PWM31, 0); //motor3
    analogWrite(PWM32, PID3); //motor1
    analogWrite(PWM41, PID4); //motor4
    analogWrite(PWM42, 0); //motor4
    //
    digitalWrite(ENM1, HIGH);
    digitalWrite(ENM2, HIGH);
    digitalWrite(ENM3, HIGH);
    digitalWrite(ENM4, HIGH);
    delay(10);

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
    delay(50);
    attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);
  } else if (buttonIndex == 2) {
    Reset_Val();
    RPM1 = 0.8646 * x - 9.9958;
    RPM2 = 0.8563 * x - 3.4796; //Target
    RPM3 = 0.8985 * x - 3.0178;
    RPM4 = 0.841 * x - 12.81;

    eror1 = RPM2 - rpm1;
    eror2 = RPM2 - rpm2;
    eror3 = RPM2 - rpm3;
    eror4 = RPM2 - rpm4;

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

    analogWrite(PWM12, 0); //motor1
    analogWrite(PWM11, PID1);
    analogWrite(PWM21, 0); //motor2
    analogWrite(PWM22, PID2); //motor1
    analogWrite(PWM31, 0); //motor3
    analogWrite(PWM32, PID3); //motor1
    analogWrite(PWM41, 0); //motor4
    analogWrite(PWM42, PID4); //motor4
    //
    digitalWrite(ENM1, HIGH);
    digitalWrite(ENM2, HIGH);
    digitalWrite(ENM3, HIGH);
    digitalWrite(ENM4, HIGH);
    digitalWrite(MUNDUR, HIGH); //BELAKANG
    delay(10);

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

    delay(50);
    attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);

  } else if (buttonIndex == 3) {
    Reset_Val();
    digitalWrite(KIRI, HIGH); //DEPAN
    RPM1 = 0.8646 * 30 - 9.9958;
    RPM2 = 0.8563 * 30 - 3.4796; //Target
    RPM3 = 0.8985 * 30 - 3.0178;
    RPM4 = 0.841 * 30 - 12.81;

    eror1 = RPM2 - rpm1;
    eror2 = RPM2 - rpm2;
    eror3 = RPM2 - rpm3;
    eror4 = RPM2 - rpm4;

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

    analogWrite(PWM12, PID1); //motor1
    analogWrite(PWM11, 0);
    analogWrite(PWM21, 0); //motor2
    analogWrite(PWM22, PID2); //motor1
    analogWrite(PWM31, PID3); //motor3
    analogWrite(PWM32, 0); //motor1
    analogWrite(PWM41, 0); //motor4
    analogWrite(PWM42, PID4); //motor4
    //
    digitalWrite(ENM1, HIGH);
    digitalWrite(ENM2, HIGH);
    digitalWrite(ENM3, HIGH);
    digitalWrite(ENM4, HIGH);
    delay(10);

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
    delay(50);
    attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
    attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);

  } else {
    digitalWrite(MAJU, LOW);
    digitalWrite(MUNDUR, LOW);
    digitalWrite(KIRI, LOW);
    digitalWrite(KANAN, LOW);

    Stop();
  }
}

/*****Maju******/
void grab_object(SerialCommands* sender) {
  digitalWrite(MAJU, HIGH);
  digitalWrite(MAJU, LOW);
  digitalWrite(MUNDUR, LOW);
  digitalWrite(KIRI, LOW);
  digitalWrite(KANAN, LOW);
  Stop();
}

// Buzzer button Callback
void sound_buzzer(SerialCommands* sender) {
  digitalWrite(BUZZER, HIGH);
  delay(1000);
  digitalWrite(BUZZER, LOW);
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
SerialCommand set_speed_command("speed", set_speed);
SerialCommand grab_object_command("grab", grab_object);
SerialCommand sound_buzzer_command("play", sound_buzzer);
SerialCommand move_servo_command("servo", move_servo);
SerialCommand right_buttons_command("button", right_buttons);

void setup() {

  vServo.attach(A0);
  hServo.attach(A0);

  vServo.write(vAngle);
  hServo.write(hAngle);

  pinMode(BUZZER, OUTPUT);
  pinMode(MAJU, OUTPUT);
  pinMode(MUNDUR, OUTPUT);
  pinMode(KIRI, OUTPUT);
  pinMode(KANAN, OUTPUT);

  Serial.begin(115200);
  HardwareSerial Serial1(PA2, PA3);

  pinMode(PWM11, OUTPUT);
  pinMode(PWM21, OUTPUT);
  pinMode(PWM31, OUTPUT);
  pinMode(PWM41, OUTPUT);

  pinMode(PWM12, OUTPUT);
  pinMode(PWM22, OUTPUT);
  pinMode(PWM32, OUTPUT);
  pinMode(PWM42, OUTPUT);

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

  serial_commands.SetDefaultHandler(cmd_unrecognized);
  serial_commands.AddCommand(&set_speed_command);
  serial_commands.AddCommand(&grab_object_command);
  serial_commands.AddCommand(&sound_buzzer_command);
  serial_commands.AddCommand(&move_servo_command);
  serial_commands.AddCommand(&right_buttons_command);

  //ultrasonic
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(ECHO3, INPUT);
  pinMode(ECHO4, INPUT);

  pinMode(TRIGGER1, OUTPUT);
  pinMode(TRIGGER2, OUTPUT);
  pinMode(TRIGGER3, OUTPUT);
  pinMode(TRIGGER4, OUTPUT);

  pinMode(BUZZER, OUTPUT);
}

void loop() {
  // Read serial commands sent by the ESP32-CAM
  serial_commands.ReadSerial();

  // Update servo positions every 50 milliseconds.
  // Using millis to avoid blocking.
  if (updateTime <= millis()) {
    // Vertical movement
    if ((vSpeed > 0 && vAngle <= 180 - vSpeed) || (vSpeed < 0 && vAngle >= abs(vSpeed))) {
      vAngle += vSpeed;
    }
    // Horizontal movement
    if ((hSpeed > 0 && hAngle <= 180 - hSpeed) || (hSpeed < 0 && hAngle >= abs(hSpeed))) {
      hAngle += hSpeed;
    }
    vServo.write(vAngle);
    hServo.write(hAngle);
    updateTime = millis() + 50;
  }

//  digitalWrite(TRIGGER1, LOW);
//  delayMicroseconds(2);
//  digitalWrite(TRIGGER1, HIGH);
//  delayMicroseconds(10);
//  duration = pulseIn(ECHO1, HIGH);
//  distance = duration * sound;
//  if (distance <= 20) {
//    digitalWrite(MAJU, HIGH);
//    delay(100);
//    digitalWrite(MAJU, LOW);
//  }
//  Serial.println(distance);

  //  digitalWrite(TRIGGER2, LOW);
  //  delayMicroseconds(2);
  //  digitalWrite(TRIGGER2, HIGH);
  //  delayMicroseconds(10);
  //  duration1 = pulseIn(ECHO2, HIGH);
  //  distance1 = duration1 * sound;
  //
  //  digitalWrite(TRIGGER3, LOW);
  //  delayMicroseconds(2);
  //  digitalWrite(TRIGGER3, HIGH);
  //  delayMicroseconds(10);
  //  duration2 = pulseIn(ECHO3, HIGH);
  //  distance2 = duration2 * sound;
  //
  //  digitalWrite(TRIGGER4, LOW);
  //  delayMicroseconds(2);
  //  digitalWrite(TRIGGER4, HIGH);
  //  delayMicroseconds(10);
  //  duration3 = pulseIn(ECHO4, HIGH);
  //  distance3 = duration3 * sound;

}

void mundur() {
  RPM1 = 0.8646 * x - 9.9958;
  RPM2 = 0.8563 * x - 3.4796; //Target
  RPM3 = 0.8985 * x - 3.0178;
  RPM4 = 0.841 * x - 12.81;

  eror1 = RPM2 - rpm1;
  eror2 = RPM2 - rpm2;
  eror3 = RPM2 - rpm3;
  eror4 = RPM2 - rpm4;

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

  analogWrite(PWM12, 0); //motor1
  analogWrite(PWM11, PID1);
  analogWrite(PWM21, 0); //motor2
  analogWrite(PWM22, PID2); //motor1
  analogWrite(PWM31, 0); //motor3
  analogWrite(PWM32, PID3); //motor1
  analogWrite(PWM41, 0); //motor4
  analogWrite(PWM42, PID4); //motor4
  //
  digitalWrite(ENM1, HIGH);
  digitalWrite(ENM2, HIGH);
  digitalWrite(ENM3, HIGH);
  digitalWrite(ENM4, HIGH);
  delay(10);

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

  delay(50);
  attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);
}

void kiri() {
  RPM1 = 0.8646 * x - 9.9958;
  RPM2 = 0.8563 * x - 3.4796; //Target
  RPM3 = 0.8985 * x - 3.0178;
  RPM4 = 0.841 * x - 12.81;

  eror1 = RPM2 - rpm1;
  eror2 = RPM2 - rpm2;
  eror3 = RPM2 - rpm3;
  eror4 = RPM2 - rpm4;

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

  analogWrite(PWM12, PID1); //motor1
  analogWrite(PWM22, PID2); //motor2
  analogWrite(PWM32, PID3); //motor3
  analogWrite(PWM42, PID4); //motor4
  //
  digitalWrite(ENM1, HIGH);
  digitalWrite(ENM2, HIGH);
  digitalWrite(ENM3, HIGH);
  digitalWrite(ENM4, HIGH);
  delay(10);

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

  delay(50);
  attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);
}

void kanan() {
  RPM1 = 0.8646 * x - 9.9958;
  RPM2 = 0.8563 * x - 3.4796; //Target
  RPM3 = 0.8985 * x - 3.0178;
  RPM4 = 0.841 * x - 12.81;

  eror1 = RPM2 - rpm1;
  eror2 = RPM2 - rpm2;
  eror3 = RPM2 - rpm3;
  eror4 = RPM2 - rpm4;

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

  analogWrite(PWM11, PID1); //motor1
  analogWrite(PWM21, PID2); //motor2
  analogWrite(PWM32, PID3); //motor3
  analogWrite(PWM42, PID4); //motor4
  //
  digitalWrite(ENM1, HIGH);
  digitalWrite(ENM2, HIGH);
  digitalWrite(ENM3, HIGH);
  digitalWrite(ENM4, HIGH);
  delay(10);

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

  delay(50);
  attachInterrupt(digitalPinToInterrupt(MENCA1), sumcounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA2), sumcounter2, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA3), sumcounter3, RISING);
  attachInterrupt(digitalPinToInterrupt(MENCA4), sumcounter4, RISING);
}

void Stop() {
  digitalWrite(ENM1, 0);
  digitalWrite(ENM2, 0);
  digitalWrite(ENM3, 0);
  digitalWrite(ENM4, 0);
  float sumeror1 = 0;
  float sumeror2 = 0;
  float sumeror3 = 0;
  float sumeror4 = 0;
  float prev1 = 0;
  float prev2 = 0;
  float prev3 = 0;
  float prev4 = 0;
  rpm1 = 0;
  rpm2 = 0;
  rpm3 = 0;
  rpm4 = 0;

}
void Reset_Val() {
  float sumeror1 = 0;
  float sumeror2 = 0;
  float sumeror3 = 0;
  float sumeror4 = 0;
  float prev1 = 0;
  float prev2 = 0;
  float prev3 = 0;
  float prev4 = 0;
  rpm1 = 0;
  rpm2 = 0;
  rpm3 = 0;
  rpm4 = 0;
}
