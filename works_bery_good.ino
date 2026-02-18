#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "DFRobot_VL53L0X.h"
#include <ServoHiwonder.hpp>
#include "rgb_lcd.h"

// Declarations
SoftwareSerial serial_motors(2, 3);  // RX, TX
ServoController servoCont(serial_motors);
DFRobot_VL53L0X sensor;
rgb_lcd lcd;

// PID vars
float setpoint = 240;   // target distance (mm)

float kp = 0.55;
float ki = 0.32;
float kd = 0.4;

const int servo_center = 600;

float error = 0;
float last_error = 0;
float last_dist = setpoint;
float integral = 0;

// Update Rate
int CRF = 100;   // control rate frequecy (Hz)
const unsigned long interval_ms = 1000 / CRF;
unsigned long lastMillis = 0;
unsigned long lastLcdUpdate = 0;
const unsigned long lcdInterval = 200;   // 5 Hz

const int min_change = 2; // Muda o salto minimo em angulo para fazer, tentei diminuir o jitter mas n fez muito, baixei para 2
static int last_pos = servo_center;

const int pinAnal = A1;  // Pino analógico para o joystick
const int pinButt = 5;  // Pino butao joystick
const int pinKi = A0;  // Pino ki
const int pinRSTKi = 4;  // reset ki
const int pinKd = A2;  // mesmo para a derivada
const int pinRSTKd = 6;  

void setup() {
  Serial.begin(115200);
  serial_motors.begin(115200);

  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  sensor.start();

  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Ola...");
  delay(200);

  pinMode(pinButt, INPUT_PULLUP);
  pinMode(pinRSTKi, INPUT_PULLUP);
  pinMode(pinRSTKd, INPUT_PULLUP);

  lastMillis = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastMillis >= interval_ms) {

    unsigned long elapsed = now - lastMillis;
    float dt = elapsed / 1000.0;
    lastMillis = now;

    //joy no stick
    int joyValue = analogRead(pinAnal);
    if (joyValue > 550) {
      setpoint += 0.1;  
    }
    else if (joyValue < 470) {
      setpoint -= 0.1;  
    }
    setpoint = constrain(setpoint, 80, 300);
    if (digitalRead(pinButt) == LOW) {
      setpoint = 240;
    }

    // controlar Ki
    joyValue = analogRead(pinKi);
    if (joyValue > 550) {
      ki += 0.001;  
    }
    else if (joyValue < 470) {
      ki -= 0.001;  
    }
    if (digitalRead(pinRSTKi) == LOW) {
      ki = 0.32;
    }

    // controlar kd
    joyValue = analogRead(pinKd);
    if (joyValue > 550) {
      kd += 0.001;  
    }
    else if (joyValue < 470) {
      kd -= 0.001;  
    }
    if (digitalRead(pinRSTKd) == LOW) {
      kd = 0.4;
    }

    float dist = sensor.getDistance();
    // fazer um filtro passa baixo do sinal para tirar jitter
    float alpha = 0.25;   // entre 0 e 1 , menor = mais filtradinho
    static float filtered_dist = 0;
    filtered_dist = alpha * dist + (1 - alpha) * filtered_dist;

    // PID
    error = setpoint - filtered_dist;

    //integral
    integral += error * dt;
    integral = constrain(integral, -400, 400);

    // derivada com filtro alfa male
    static float derivative = 0;
    const float alpha_d = 0.25;
    float d_raw = -(filtered_dist - last_dist) / dt;
    derivative = alpha_d * d_raw + (1 - alpha_d) * derivative;

    float pid = kp * error + ki * integral + kd * derivative;

    last_error = error;
    last_dist = filtered_dist;

    int pos_motor = (int)(servo_center - pid);
    pos_motor = constrain(pos_motor, servo_center - 300, servo_center + 300);
    if (abs(pos_motor - last_pos) >= min_change) {
      servoCont.moveWithTime(1, pos_motor, interval_ms);
      last_pos = pos_motor;
    }

    // Serial debug para o serial plotter do arduino
    Serial.print(0);
    Serial.print(",");
    Serial.print(350);
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(dist);
    Serial.print(",");
    Serial.println(filtered_dist);

    // LCD
    if (now - lastLcdUpdate >= lcdInterval) {
      lastLcdUpdate = now;

      lcd.setCursor(0, 0);
      lcd.print("D:");
      lcd.print((int)filtered_dist);
      lcd.print(" S:");
      lcd.print((int)setpoint);
      lcd.print("   ");              

      lcd.setCursor(0, 1);
      lcd.print("Ki:");
      lcd.print(ki, 2);
      lcd.print(" Kd:");
      lcd.print(kd, 2);
      lcd.print(" ");                  
    }
  }

}
