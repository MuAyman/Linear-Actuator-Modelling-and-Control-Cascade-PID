#define ENCA 2
#define ENCB 4
#define LIMIT_SWITCH_PIN 3
#define PWM 5
#define IN1 6
#define IN2 7
#include "TimerOne.h"
#include <PID_v1.h>




////////////////////
// globals
long prevT = 0;
int flag = 0;
double posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
int pos = 0;
double vel = 0;
double setPointPosition = 0;
double setPointVelocity = 0;
double pwr = 0;

double pulses = 0;
int Bar = 82;
int pwrPrev = 0;
double currVel = 0;
double currPos = 0;
int counts = 0;
bool gettingNewSetpoint = false;
int dir = -1;
PID PosPID(&currPos, &setPointVelocity, &setPointPosition, 9.5, 0, 0, DIRECT);
PID VelPID(&currVel, &pwr, &setPointVelocity, 20, 600, 0, DIRECT);
int currSetPos = 0;
int barflag = 0;
int limitflag = 0;


void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  Serial.print(digitalRead(LIMIT_SWITCH_PIN));
  homemotor();
  setMotor(1, 0, PWM, IN1, IN2);
  PosPID.SetMode(AUTOMATIC);
  VelPID.SetMode(AUTOMATIC);
  PosPID.SetOutputLimits(-2, 2);
  VelPID.SetOutputLimits(-255, 255);
  PosPID.SetSampleTime(10);
  VelPID.SetSampleTime(10);

  setPointPosition = getSetpointFromUser();
  currSetPos = setPointPosition;
  limitflag = 1;  
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), zeroing, FALLING);
  Timer1.initialize(1000000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(dataAqu, 10000);

}

void loop() {
  if (currPos >= Bar) {
    if (barflag == 0) {
      setPointPosition = 2 * Bar - setPointPosition;
      barflag = 1;
      limitflag = 0;
    }
  }

  if (flag == 100) {

    if (!gettingNewSetpoint) {
      gettingNewSetpoint = true;
      currSetPos = getSetpointFromUser();
      setPointPosition = setPointPosition + currSetPos;
      //setPointVelocity = setPointPosition - currPos;
      gettingNewSetpoint = false;
      flag = 0;
      barflag = 0;
      limitflag = 0;
    }
  }
}

void zeroing() {
  if (!digitalRead(LIMIT_SWITCH_PIN)) {
    if (limitflag == 0) {
      setPointPosition = -1 * setPointPosition;
      limitflag = 1;
    }
  }
}

void dataAqu()
{
  if (!gettingNewSetpoint) {
    int pos = 0;

    noInterrupts(); // disable interrupts temporarily while reading
    pos = counts;
    vel = velocity_i;
    interrupts();
   
    currVel = vel * 0.0446555;
    currPos = pos * 0.0446555;

    if (pwr == 255) {VelPID.SetTunings(20, 0, 0);}
    else {VelPID.SetTunings(20, 200, 0);}

    PosPID.Compute();
    VelPID.Compute();
    if (pos == posPrev) {
      currVel = 0;
    }
    pwrPrev = pwr;
    posPrev = pos;

    if (pwr < 67 && pwr > -67){pwr = 0;}
    if (pwr == 0)
    {flag = flag + 1;}
    else {flag = 0;}

    setMotor(dir, pwr, PWM, IN1, IN2);
    
    
    
    Serial.print(pwr);
    Serial.print("\t");
    Serial.print(pos);
    Serial.print("\t");
    Serial.print(currPos);
    Serial.print("\t");
    Serial.println(currVel);
  }






}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  if (pwmVal >= 0) {
    analogWrite(pwm, pwmVal); // Motor speed
    if (dir == 1) {
      // Turn one way
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    else if (dir == -1) {
      // Turn the other way
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else {
      // Or dont turn
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
  }
  if (pwmVal < 0) {
    pwmVal = pwmVal * -1;
    analogWrite(pwm, pwmVal); // Motor speed
    if (dir == -1) {
      // Turn one way
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    else if (dir == 1) {
      // Turn the other way
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else {
      // Or dont turn
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
    pwmVal = pwmVal * -1;
  }

}

float getSetpointFromUser() {
  Serial.print("Enter the new setpoint (in cm): ");
  while (Serial.available() == 0) {};
  float setpoint = float(Serial.parseInt());
  Serial.print("New setpoint: ");
  Serial.println(setpoint);
  return setpoint;
}

void homemotor() {
  while (digitalRead(LIMIT_SWITCH_PIN) == HIGH && Serial.available() == 0) {
    setMotor(1, 200, PWM, IN1, IN2);
  }
}
void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0) {
    // If B is high, increment forward
    increment = 1;
    counts++;
  }
  else {
    // Otherwise, increment backward
    increment = -1;
    counts--;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i)) / 1.0e6;
  velocity_i = increment / deltaT;
  prevT_i = currT;
}
