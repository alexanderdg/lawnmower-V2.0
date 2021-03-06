#include<Arduino.h>
#include "MotorDriver.h"
#include <PID_v1.h>


MotorDriver* MotorDriver::MotorDriverS = 0;

MotorDriver::MotorDriver() : LPID(&InputL, &OutputL, &SetpointL, 5, 40, 0.5, DIRECT), RPID(&InputR, &OutputR, &SetpointR, 5, 40, 0.5, DIRECT) {
  MotorDriverS = this;
  analogWriteResolution(12);
  analogReadResolution(12);
  analogWriteFrequency(L_PWM, 20000);
  analogWriteFrequency(R_PWM, 20000);
  pinMode(RL_EN, OUTPUT);
  pinMode(RL_SLEEP, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(B_EN, OUTPUT);
  pinMode(B_SLEEP, OUTPUT);
  pinMode(B_DIR, OUTPUT);
  pinMode(B_PWM, OUTPUT);
  pinMode(L_CURRENT, INPUT);
  pinMode(R_CURRENT, INPUT);
  pinMode(EN_VBLADE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENCA), MotorDriver::EncoderInteruptL, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCA), MotorDriver::EncoderInteruptR, RISING);
  resetCounterTimer.begin(MotorDriver::resetCounter, 100000);
  sampleCurrentTimer.begin(MotorDriver::currentSampler, 10000);
  LPID.SetOutputLimits(0, 4095);
  RPID.SetOutputLimits(0, 4095);
  LPID.SetMode(AUTOMATIC);
  RPID.SetMode(AUTOMATIC);
}

bool MotorDriver::selfTest(void) {
  bool temp = false;
  total_counter_l = 0;
  total_counter_r = 0;
  Forward(0.3);
  delay(500);
  coastBrake();
  if ((total_counter_l > 0) && (total_counter_r > 0)) temp = true;
  return temp;
}

bool MotorDriver::Forward(float speed) {
  digitalWrite(RL_EN, LOW);
  digitalWrite(RL_SLEEP, HIGH);
  digitalWrite(R_DIR, HIGH);
  digitalWrite(L_DIR, LOW);
  SetpointL = MStoCPMSconverter(speed);
  SetpointR = MStoCPMSconverter(speed);
  en_speed_control = true;
  en_distance_control = false;
  LPID.SetMode(AUTOMATIC);
  RPID.SetMode(AUTOMATIC);
  return true;
}

bool MotorDriver::Backward(float speed) {
  digitalWrite(RL_EN, LOW);
  digitalWrite(RL_SLEEP, HIGH);
  digitalWrite(R_DIR, LOW);
  digitalWrite(L_DIR, HIGH);
  SetpointL = MStoCPMSconverter(speed);
  SetpointR = MStoCPMSconverter(speed);
  en_speed_control = true;
  en_distance_control = false;
  LPID.SetMode(AUTOMATIC);
  RPID.SetMode(AUTOMATIC);
  return true;
}

bool MotorDriver::turnLeft(float speed) {
  digitalWrite(RL_EN, LOW);
  digitalWrite(RL_SLEEP, HIGH);
  digitalWrite(R_DIR, HIGH);
  digitalWrite(L_DIR, HIGH);
  SetpointL = MStoCPMSconverter(speed);
  SetpointR = MStoCPMSconverter(speed);
  en_speed_control = true;
  en_distance_control = false;
  LPID.SetMode(AUTOMATIC);
  RPID.SetMode(AUTOMATIC);
  return true;
}

bool MotorDriver::turnRight(float speed) {
  digitalWrite(RL_EN, LOW);
  digitalWrite(RL_SLEEP, HIGH);
  digitalWrite(R_DIR, LOW);
  digitalWrite(L_DIR, LOW);
  SetpointL = MStoCPMSconverter(speed);
  SetpointR = MStoCPMSconverter(speed);
  en_speed_control = true;
  en_distance_control = false;
  LPID.SetMode(AUTOMATIC);
  RPID.SetMode(AUTOMATIC);
  return true;
}

bool MotorDriver::rawRight(bool direction, int pwm) {
  digitalWrite(RL_EN, LOW);
  digitalWrite(RL_SLEEP, HIGH);
  digitalWrite(R_DIR, direction);
  en_speed_control = false;
  en_distance_control = false;
  SetpointL = 0;
  SetpointR = 0;
  OutputL = 0;
  OutputR = 0;
  LPID.SetMode(MANUAL);
  RPID.SetMode(MANUAL);
  setRightSpeed(pwm);
  return true;
}

bool MotorDriver::rawRight(int pwm) {
  if(pwm >= 0) rawRight(0, pwm);
  else rawRight(1, pwm);
  return true;
}

bool MotorDriver::rawLeft(bool direction, int pwm) {
  digitalWrite(RL_EN, LOW);
  digitalWrite(RL_SLEEP, HIGH);
  digitalWrite(L_DIR, direction);
  en_speed_control = false;
  en_distance_control = false;
  SetpointL = 0;
  SetpointR = 0;
  OutputL = 0;
  OutputR = 0;
  LPID.SetMode(MANUAL);
  RPID.SetMode(MANUAL);
  setLeftSpeed(pwm);
  return true;
}

bool MotorDriver::rawLeft(int pwm) {
  if(pwm >= 0) rawLeft(0, pwm);
  else rawLeft(1, pwm);
  return true;
}

bool MotorDriver::takeTurnRight(float speed, int angle) {
  bool returnvalue = false;
  if (angle >= 90) {
    //calculations to convert angle to encoder values
    int distance_mm = (angle / 360.0) * 3.1415 * DISTANCE_BETWEEN_WHEELS;
    int value_encoder = MStoCPMSconverter(distance_mm / 100.0);
    turnRight(speed);
    distance_start_value = total_counter_l;
    distance_target = value_encoder;
    en_distance_control = true;
    distance_target_reached = false;
  }
  return returnvalue;
}

bool MotorDriver::takeTurnLeft(float speed, int angle) {
  bool returnvalue = false;
  if (angle >= 90) {
    //calculations to convert angle to encoder values
    int distance_mm = (angle / 360.0) * 3.1415 * DISTANCE_BETWEEN_WHEELS;
    int value_encoder = MStoCPMSconverter(distance_mm / 100.0);
    turnLeft(speed);
    distance_start_value = total_counter_l;
    distance_target = value_encoder;
    en_distance_control = true;
    distance_target_reached = false;
  }
  return returnvalue;
}

bool MotorDriver::driveForward(float speed, float distance) {
  bool returnvalue = false;
  if (distance >= 0.2) {
    int value_encoder = MStoCPMSconverter(distance * 10);
    Forward(speed);
    distance_start_value = total_counter_l;
    distance_target = value_encoder;
    en_distance_control = true;
    distance_target_reached = false;
  }
  return returnvalue;
}

bool MotorDriver::driveBackward(float speed, float distance) {
  bool returnvalue = false;
  if (distance >= 0.2) {
    int value_encoder = MStoCPMSconverter(distance * 10);
    Backward(speed);
    distance_start_value = total_counter_l;
    distance_target = value_encoder;
    en_distance_control = true;
    distance_target_reached = false;
  }
  return returnvalue;
}

bool MotorDriver::coastBrake(void) {
  en_speed_control = false;
  en_distance_control = false;
  SetpointL = 0;
  SetpointR = 0;
  OutputL = 0;
  OutputR = 0;
  LPID.SetMode(MANUAL);
  RPID.SetMode(MANUAL);
  digitalWrite(RL_EN, HIGH);
  setRightSpeed(0);
  setLeftSpeed(0);
  return true;
}

bool MotorDriver::dynamicBrake(void) {
  en_speed_control = false;
  en_distance_control = false;
  SetpointL = 0;
  SetpointR = 0;
  OutputL = 0;
  OutputR = 0;
  LPID.SetMode(MANUAL);
  RPID.SetMode(MANUAL);
  setLeftSpeed(0);
  setRightSpeed(0);
  return true;
}

bool MotorDriver::takeRandomTurnRight(void) {
  turnRight(0.5);
  delay(random(500, 2000));
  coastBrake();
  return true;
}

bool MotorDriver::takeRandomTurnLeft(void) {
  turnLeft(0.5);
  delay(random(500, 2000));
  coastBrake();
  return true;
}

int MotorDriver::MStoCPMSconverter(float speed) {
  float RPS = speed / perimeterWheel;
  int CountEncoder = RPS * (ENCODER_COUNTS_PER_REVOLUTION / 4);
  return CountEncoder / 10;
}

float MotorDriver::CPMStoMSconverter(int speed) {
  int countS = speed * 10;
  float RPS = countS / (ENCODER_COUNTS_PER_REVOLUTION / 4);
  float speedS = RPS * perimeterWheel;
  return speedS;
}

void MotorDriver::setLeftSpeed(int speed) {
  analogWrite(L_PWM, speed);
}

void MotorDriver::setRightSpeed(int speed) {
  analogWrite(R_PWM, speed);
}

bool MotorDriver::ForwardLeft(float speed) {
  digitalWrite(RL_EN, LOW);
  digitalWrite(RL_SLEEP, HIGH);
  digitalWrite(L_DIR, LOW);
  SetpointL = MStoCPMSconverter(speed);
  en_speed_control = true;
  en_distance_control = false;
  LPID.SetMode(AUTOMATIC);
  return true;
}

bool MotorDriver::ForwardRight(float speed) {
  digitalWrite(RL_EN, LOW);
  digitalWrite(RL_SLEEP, HIGH);
  digitalWrite(R_DIR, HIGH);
  SetpointR = MStoCPMSconverter(speed);
  en_speed_control = true;
  en_distance_control = false;
  RPID.SetMode(AUTOMATIC);
  return true;
}

float MotorDriver::getLeftSpeed(void) {
  return CPMStoMSconverter(counter_l);
}

float MotorDriver::getRightSpeed(void) {
  return CPMStoMSconverter(counter_r);
}

float MotorDriver::getLeftCurrent(void) {
  float temp = (leftCurrent * 0.000806) * 5;
  return temp;
}

float MotorDriver::getRightCurrent(void) {
  float temp = (rightCurrent * 0.000806) * 5;
  return temp;
}

bool MotorDriver::distanceTargetReached(void) {
  return distance_target_reached;
}

bool MotorDriver::enableVBlade(void) {
  digitalWrite(EN_VBLADE, HIGH);
  return true;
}

bool MotorDriver::disableVBlade(void) {
  digitalWrite(EN_VBLADE, LOW);
  return true;
}

bool MotorDriver::startTurning(void) {
  digitalWrite(B_SLEEP, HIGH);
  digitalWrite(B_EN, LOW);
  digitalWrite(B_DIR, LOW);
  digitalWrite(B_PWM, HIGH);
  return true;
}

bool MotorDriver::bladeStop(void) {
  digitalWrite(B_SLEEP, HIGH);
  digitalWrite(B_EN, HIGH);
  digitalWrite(B_DIR, LOW);
  digitalWrite(B_PWM, LOW);
  return true;
}

bool MotorDriver::bladeEmergencyStop(void) {
  digitalWrite(B_SLEEP, HIGH);
  digitalWrite(B_EN, LOW);
  digitalWrite(B_DIR, LOW);
  digitalWrite(B_PWM, LOW);
  return true;
}

float MotorDriver::getBladeCurrent(void) {
  //current is the sum of the two HS mosfet so there is a difference between the actual motor current
  //the actual motor current is lower depeding on the PWM dutycycle and can be calculated
  //formula is imotor = (i x d) + ((1 - d) x i x 0.5)
  //because the current is relativly used the motor current isn't calculated
  float temp = (bladeCurrent * 0.000806) * 5;
  return temp;
}

//-------------------------------------------------------------------------------------
//                  Interupt routines
//-------------------------------------------------------------------------------------

void MotorDriver::EncoderInteruptL() {
  MotorDriverS -> temp_counter_l ++;
  MotorDriverS -> total_counter_l ++;
}

void MotorDriver::EncoderInteruptR() {
  MotorDriverS -> temp_counter_r ++;
  MotorDriverS -> total_counter_r ++;
}

void MotorDriver::resetCounter() {
  MotorDriverS -> counter_l = MotorDriverS -> temp_counter_l;
  MotorDriverS -> counter_r = MotorDriverS -> temp_counter_r;
  MotorDriverS -> temp_counter_l = 0;
  MotorDriverS -> temp_counter_r = 0;
  MotorDriverS -> InputL = MotorDriverS -> counter_l;
  MotorDriverS -> InputR = MotorDriverS -> counter_r;
  MotorDriverS -> LPID.Compute();
  MotorDriverS -> RPID.Compute();
  if (MotorDriverS -> en_speed_control)
  {
    MotorDriverS -> setLeftSpeed(MotorDriverS -> OutputL);
    MotorDriverS -> setRightSpeed(MotorDriverS -> OutputR);
    //MotorDriverS -> setLeftSpeed(2800);
    //MotorDriverS -> setRightSpeed(2800);
  }
  else
  {
    MotorDriverS -> SetpointL = 0;
    MotorDriverS -> SetpointR = 0;
    //MotorDriverS -> setLeftSpeed(0);
    //MotorDriverS -> setRightSpeed(0);
    //Serial.println(MotorDriverS -> OutputL);
    //Serial.println(MotorDriverS -> OutputR);
  }

  if (MotorDriverS -> en_distance_control)
  {
    int last_distance  = MotorDriverS -> total_counter_l - MotorDriverS -> distance_start_value;
    if (last_distance >=  MotorDriverS -> distance_target)
    {
      MotorDriverS -> coastBrake();
      MotorDriverS -> distance_target_reached = true;
      Serial3.println("Distance target reached!");
    }
  }
  /*
    Serial.print("Input PID: ");
    Serial.println(MotorDriverS -> counter_l);
    Serial.println("Setpoint PID: ");
    Serial.println(MotorDriverS -> SetpointL);
    Serial.println("Output PID: ");
    Serial.println(MotorDriverS -> OutputL);
  */
}

void MotorDriver::currentSampler() {
  int temp = analogRead(MotorDriverS -> L_CURRENT);
  MotorDriverS -> leftCurrent = (0.9 * MotorDriverS -> leftCurrent) +  (0.1 * temp);
  temp = analogRead(MotorDriverS -> R_CURRENT);
  MotorDriverS -> rightCurrent = (0.9 * MotorDriverS -> rightCurrent) +  (0.1 * temp);
  temp = analogRead(MotorDriverS -> B_CURRENT);
  MotorDriverS -> bladeCurrent = (0.9 * MotorDriverS -> bladeCurrent) +  (0.1 * temp);
}
