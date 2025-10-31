#include <SoftwareSerial.h>
#include "DFRobot_VL53L0X.h"
#include "rgb_lcd.h"          //its the Grove - LCD Backlight    by Seeed
#include <ServoHiwonder.hpp>

SoftwareSerial serial_motors(2, 3); // RX, TX , Só estou a usar TX
ServoController servoCont(serial_motors);
DFRobot_VL53L0X sensor;

float setpoint = 250; // distancia objetivo
float Kp = 0.6;
float Ki = 0.0;
float Kd = 0.06;

// Servo
const float servoCenter = 660.0;
int servoMin = servoCenter - 120;   // posições maximas permitidas ao servo
int servoMax = servoCenter + 120;  


float last_error = 0;
float integral = 0;
float last_servo_cmd = servoCenter;  // store last commanded position - posiçao do meio

int Fs = 1000; // Sample Rate
 

//  Smoothing do movimento
float maxStepPerCycle = 10.0; // steps maximos por ciclo/periodo

float derivativeFilterAlpha = 0.7; // Filtro derivativo -- 0.0–1.0 (maior = mais smoothing)
float derivativeFiltered = 0; // 

float errorDeadband = 15.0; // mm

int count = 0;

rgb_lcd lcd;

float last_dist = 0;
unsigned long last_time = 0;

void setup() {
  Serial.begin(115200);
  serial_motors.begin(115200);
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  sensor.start();

  lcd.begin(16, 2);

  lcd.blinkLED();

  lcd.begin(0, 0);  // begin do lcd
  lcd.setCursor(0, 0);
  lcd.print("NEEEC - FEUP");  // printzao basico no cursor
  lcd.setCursor(0, 1);
  
  delay(200);
}

void loop() {

  static unsigned long lastMillis = 0;
  unsigned long period = 1000 / Fs;
  if (millis() - lastMillis < period) return;
  lastMillis = millis();

  float dist = sensor.getDistance(); // mm
  float error = setpoint - dist;
  // deadzone ajuda contra o jitter
  if (abs(error) < errorDeadband + errorDeadband) error = 0;

  float dt = period / 1000.0;

  // --- PID ---
  float P = Kp * error;

  integral += error * dt;
  integral = constrain(integral, -50.0, 50.0);
  float I = Ki * integral;

  float derivative = (error - last_error) / dt;
  derivativeFiltered = derivativeFilterAlpha * derivativeFiltered +
                       (1 - derivativeFilterAlpha) * derivative;
  float D = Kd * derivativeFiltered;

  float pid = (P + I + D);

  if(dist > setpoint) pid *= 1.1;
  // Associar Pid a posiçao do servo
  float targetPos = servoCenter - pid;

  // limitar range
  targetPos = constrain(targetPos, servoMin, servoMax);

  // smoothing do movimento
  float delta = targetPos - last_servo_cmd;
  if (delta > maxStepPerCycle) delta = maxStepPerCycle;
  if (delta < -maxStepPerCycle) delta = -maxStepPerCycle;
  float smoothedPos = last_servo_cmd + delta;

  // clamp
  smoothedPos = constrain(smoothedPos, servoMin, servoMax);

  int posMotor = (int)smoothedPos;
  servoCont.moveWithTime(1, posMotor, 300 / Fs);

  last_servo_cmd = smoothedPos;
  last_error = error;

  // prints
  

  if (count >= 40){
    lcd.setCursor(0, 0);
    lcd.print("NEEEC - FEUP");  // print a simple message
    lcd.setCursor(0, 1);
    lcd.print(dist);
    lcd.setCursor(7, 1);
    lcd.print("mm");

    Serial.print("dist(mm): "); Serial.print(dist);
    Serial.print(" erro: "); Serial.print(error);
    Serial.print(" pid: "); Serial.print(pid);
    Serial.print(" count: "); Serial.print(count);
    Serial.print(" pos: "); Serial.println(posMotor);

    count = 0;
  }

  count++;
  if(dist == 16383.75 && dist != last_dist)
  {
    last_time = millis();
  }

  if(dist == 16383.75 && last_dist == dist && millis() - last_time >= 5000){
    sensor.begin(0x50);
    sensor.setMode(sensor.eContinuous, sensor.eHigh);
    sensor.start();
    last_time = millis();
  }
  last_dist = dist;
}