#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

int encoderPinA[] = {2, 3, 19};
int encoderPinB[] = {5, 4, 23};
int MotorPWM[]    = {8, 9, 10};
int MotorPin1[]   = {7, 6, 29};
int MotorPin2[]   = {12, 11, 33};

long    counts[3]   = {};
float   act_theta[3]     = {};
float   prv_theta[3] = {};
double  des_theta[3] = {};

double  now_time, samp_time, prv_time;
double  err[3]      = {};
double  prv_err[3]  = {};
double  eint[3]     = {};
double  prv_eint[3] = {};
double  Volt[3]     = {};
//double  Kp[3]       = {0.8  , 0.5  ,0.5};
//double  Kd[3]       = {0.02 , 0.02 ,0.02};
//double  Ki[3]       = {0.001, 0.004  ,0.004};
double  Kp[3]       = {0.8  , 3.0  , 0.5};
double  Kd[3]       = {0.02 , 0.05 , 0.02};
double  Ki[3]       = {0.001, 2.0  , 0.004};
//double  Kp[3]       = {0.8,1.7,0.8};
//double  Kd[3]       = {0.02,0.05,0.03};
//double  Ki[3]       = {0.001,0.001,0.002};
double  PWM[3]      = {};
double  GR[3]       = {100.0, 75.0, 75.0};
double  cpr[3]      = {64, 48, 48};

double theta[3] = {};
double theta_1, theta_2, theta_3;
double des_theta1, des_theta2, des_theta3;
double prv_des_theta1 = 0;
double prv_des_theta2 = 0;
double prv_des_theta3 = 0;
double tick = 0;

void setup() {
  Serial.begin(115200);

  for (int ii = 0; ii < 3; ii++) {

    pinMode(encoderPinA[ii], INPUT);
    pinMode(encoderPinB[ii], INPUT);
    pinMode(MotorPWM[ii], OUTPUT);
    pinMode(MotorPin1[ii], OUTPUT);
    pinMode(MotorPin1[ii], OUTPUT);
    digitalWrite(encoderPinA[ii], LOW);
    digitalWrite(encoderPinB[ii], LOW);
    attachInterrupt(digitalPinToInterrupt(encoderPinA[0]), readEncoder0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinA[1]), readEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinA[2]), readEncoder2, CHANGE);
  }
}

void loop() {
  if ( Serial.available() )
  {
    String  matlabdata = Serial.readString();
    String  matlabdata_1    =  matlabdata.substring(0, 4);
    int     data_1    =  matlabdata_1.toInt();
    String  matlabdata_2    =  matlabdata.substring(5, 9);
    int     data_2    =  matlabdata_2.toInt();
    String  matlabdata_3    =  matlabdata.substring(10, 14);
    int     data_3    =  matlabdata_3.toInt();
    
    prv_des_theta1 = des_theta1;
    prv_des_theta2 = des_theta2;
    prv_des_theta3 = des_theta3;    

    des_theta1 = data_1;
    des_theta2 = data_2;
    des_theta3 = data_3;
    tick = 0;

  }

  do
  {
    tick++;
    if (tick < 500)
    {
      des_theta[0] = prv_des_theta1 + ((des_theta1 - prv_des_theta1) / 500.0) * tick;
      des_theta[1] = prv_des_theta2 + ((des_theta2 - prv_des_theta2) / 500.0) * tick;
      des_theta[2] = prv_des_theta3 + ((des_theta3 - prv_des_theta3) / 500.0) * tick;
    }
  else
  {
      des_theta[0] = des_theta1;
      des_theta[1] = des_theta2;
      des_theta[2] = des_theta3;
    }
    for (int x = 0; x < 3; x++) {

      now_time  = micros() / 1000000.0;
      samp_time = now_time - prv_time;
      prv_time  = now_time;
      act_theta[x]      = ((360.0 * 2) / (cpr[x] * GR[x])) * counts[x];
      err[x]        = des_theta[x] - act_theta[x];
      eint[x]       = err[x] * samp_time + prv_eint[x];
      Volt[x]       = Kp[x] * err[x] + (Kd[x] * ( (err[x] - prv_err[x]) / samp_time )) + Ki[x] * eint[x];
      PWM[x]        = (abs(Volt[x]) / 12.0) * 255;
      if ( PWM[x] > 255 ) {
        PWM[x] = 255;
      }
      else             {
        PWM[x] = PWM[x];
      }

      if (Volt[x] > 0)  {
        digitalWrite(MotorPin1[x], HIGH);
        digitalWrite(MotorPin2[x], LOW);
      }

      else           {
        digitalWrite(MotorPin1[x], LOW);
        digitalWrite(MotorPin2[x], HIGH);
      }
      analogWrite (MotorPWM[x], PWM[x]);//PWM
      prv_theta[x]  = act_theta[x];
      prv_err[x]    = err[x];
      prv_eint[x]   = eint[x];

    }
  } while (abs(err[0]) > 10.0 && abs(err[1]) > 10.0 && abs(err[2]) > 10.0);
}
//-------------------------------------------
void readEncoder0() {
  if (digitalRead(encoderPinB[0]) == digitalRead(encoderPinA[0])) {
    counts[0] = counts[0] + 1;
  }
  else                                                              {
    counts[0] = counts[0] - 1;
  }
}
void readEncoder1() {
  if (digitalRead(encoderPinB[1]) == digitalRead(encoderPinA[1])) {
    counts[1] = counts[1] + 1;
  }
  else                                                              {
    counts[1] = counts[1] - 1;
  }
}
void readEncoder2() {
  if (digitalRead(encoderPinB[2]) == digitalRead(encoderPinA[2])) {
    counts[2] = counts[2] + 1;
  }
  else                                                              {
    counts[2] = counts[2] - 1;
  }
}
//-------------------------------------------
