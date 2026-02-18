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
float setpoint = 250;   // target distance (mm)

float kp = 0.55;
float ki = 0.32;
float kd = 0.3;

const int servo_center = 600;

float error = 0;
float last_error = 0;
float integral = 0;

// Control rate
int CRF = 500;   //
unsigned long interval_ms = 1000 / CRF;
unsigned long lastMillis = 0;

const int min_change = 15;
static int last_pos = servo_center;

const int pinAnal = A1;  // Pino analógico para o joystick
const int pinButt = 5;  // Pino butao joystick

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
      setpoint += 1;  // increase target
    }
    else if (joyValue < 470) {
      setpoint -= 1;  // decrease target
    }
    setpoint = constrain(setpoint, 50, 500);

    if (digitalRead(pinButt) == LOW) {
      setpoint = 250;
    }

    float dist = sensor.getDistance();
    float alpha = 0.25;   // entre 0 e 1 , menor = mais filter
    static float filtered_dist = 0;
    filtered_dist = alpha * dist + (1 - alpha) * filtered_dist;

    // PID
    error = setpoint - filtered_dist;

    integral += error * dt;
    integral = constrain(integral, -400, 400);

    float derivative = 0;
    if (dt > 0.00001)
      derivative = (error - last_error) / dt;

    float pid = kp * error + ki * integral + kd * derivative;

    last_error = error;

    int pos_motor = (int)(servo_center - pid);
    pos_motor = constrain(pos_motor, servo_center - 300, servo_center + 300);
    if (abs(pos_motor - last_pos) >= min_change) {
      servoCont.moveWithTime(1, pos_motor, interval_ms);
      last_pos = pos_motor;
    }

    // Serial debug
    // --- For Serial Plotter ---
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
    lcd.setCursor(0, 0);
    lcd.print("Dist:");
    lcd.print((int)dist);
    lcd.print("mm   ");

    lcd.setCursor(0, 1);
    lcd.print("Setpoint:");
    lcd.print((int)setpoint);
    lcd.print(" ");
  }

}
