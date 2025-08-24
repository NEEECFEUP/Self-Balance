#include <ServoHiwonder.hpp>
#include <ServoHiwonderClass.hpp>

#include "Arduino.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include "rgb_lcd.h"          //its the Grove - LCD Backlight    by Seeed
#include "DFRobot_VL53L0X.h"  //its the DFRobot_VL53L0X          by DFRobot




/*
  Project Name: Self balancing
  
  Description:
  This program is designed to control a motor using a PID (Proportional-Integral-Derivative) controller, which adjusts the motor position based on the distance of an object. 
  The setup integrates an Arduino-compatible microcontroller, a VL53L0X laser distance sensor for precise range detection, and a servo motor for movement.

  Key Features:
  - PID controller that dynamically updates motor position to maintain a setpoint distance.
  - Real-time adjustable PID gains (Kp, Ki, Kd) using joystick analog inputs.
  - Reset functionality for PID gains via dedicated buttons.
  - LCD display showing current PID values and system information.
  - Communication with the servo motor through serial commands to perform controlled movements.
  - Laser distance measurement in high-precision continuous mode.

   Components & Wiring:
  - VL53L0X Distance Sensor:
    - SDA: A4 (Arduino Uno)
    - SCL: A5 (Arduino Uno)
    - VIN: 5V
    - GND: GND
  - RGB LCD Display:
    - SDA: A4 (shared with VL53L0X)
    - SCL: A5 (shared with VL53L0X)
    - VIN: 5V
    - GND: GND
  - Joystick for PID Adjustment:
    - Kp: A0 (Proportional gain)
    - Ki: A1 (Integral gain)
    - Kd: A2 (Derivative gain)
  - Reset Buttons:
    - Kp Reset: Pin 4
    - Ki Reset: Pin 5
    - Kd Reset: Pin 6
  - Servo Motor:
    - Data: Using serial at pins 2 and 3 (Rx, TX) and only Tx is used
    - VIN: 5V
    - GND: GND

  Note:
  Adjustments to the control and update rate frequencies (CRF, URF) can refine the system's responsiveness.
  Adjustments to the target position can be made using variable setpoint.
  Adjustments to the default PID can be made using variables kp,ki and kd.
*/




SoftwareSerial serial_motors(2, 3);  // RX, TX

ServoController servoCont(serial_motors);

int CRF = 20;  //control rate frequecy
int URF = 5;   //update rate frequency

float radius = 20;  //raio da bola para fazer a diferença do centro de massa e da deteção do sensor
float setpoint = 240/*18*/;  //distancia a que deve estar a bola (centro da barra)
float kp = 2.4/*24*/;
float ki = 0.06/*0.6*/;
float kd = 9.8/*98*/;


int loop_nr = 0;
DFRobot_VL53L0X sensor;
rgb_lcd lcd;


int max_change_per_cycle = 40;  //equals to 10 degrees per 50ms

float dist = 0;
float last_error = 0;
float error = 0;
float changeError = 0;
float totalError = 0;
float pidTerm = 0;
float last_pidTerm = 0;
int pos_motor = 0;
float pidTerm_scaled = 0;  //if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |1000|

float x1 = 500;
float x2 = 0;
float u = 0;
float error_sim = 0;
float last_error_Sim = 0;
float i_error_Sim = 0;
float d_error_Sim = 0;

char message[200];
// Define os pinos do joystick
const int pinKp = A0;  // Pino analógico para o Kp
const int pinKi = A1;  // Pino analógico para o Ki
const int pinKd = A2;  // Pino analógico para o Kd

const int pinRSTKp = 4;  // Pino reset para o Kp
const int pinRSTKi = 5;  // Pino reset para o Ki
const int pinRSTKd = 6;  // Pino reset para o Kd

float val_inc_kp = 0, val_inc_ki = 0, val_inc_kd = 0;
float inc_kp = 0, inc_kd = 0, inc_ki = 0;

float big_move_th = 0.25;
float zero_move_th = 0.1;
float big_inc = 0.05;
float small_inc = 0.02;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  serial_motors.begin(115200);
    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);

  lcd.blinkLED();

  lcd.begin(0, 0);  // start the library
  lcd.setCursor(0, 0);
  lcd.print("Heil Migo,");  // print a simple message
  lcd.setCursor(0, 1);
  lcd.print("meu amigo");  // print a simple message

  //join i2c bus (address optional for master)
  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  //Laser rangefinder begins to work
  sensor.start();

  pinMode(pinRSTKp, INPUT);
  pinMode(pinRSTKd, INPUT);
  pinMode(pinRSTKi, INPUT);

  lcd.blink();
  delay(1000);
  lcd.blinkLED();
  lcd.clear();

  do{x1 = sensor.getDistance();}while(x1 >= 480); // 480/500 checar um valor util
}


void loop() {

  if (0 == loop_nr % (int)(CRF / URF)) {

    if (analogRead(pinKp) < big_move_th * 1024)
      val_inc_kp -= big_inc;
    else if (analogRead(pinKp) < (0.5 - zero_move_th) * 1024)
      val_inc_kp -= small_inc;
    else if (analogRead(pinKp) > (1 - big_move_th) * 1024)
      val_inc_kp += big_inc;
    else if (analogRead(pinKp) > (0.5 + zero_move_th) * 1024)
      val_inc_kp += small_inc;

    if (analogRead(pinKi) < big_move_th * 1024)
      val_inc_ki -= big_inc;
    else if (analogRead(pinKi) < (0.5 - zero_move_th) * 1024)
      val_inc_ki -= small_inc;
    else if (analogRead(pinKi) > (1 - big_move_th) * 1024)
      val_inc_ki += big_inc;
    else if (analogRead(pinKi) > (0.5 + zero_move_th) * 1024)
      val_inc_ki += small_inc;

    if (analogRead(pinKd) < big_move_th * 1024)
      val_inc_kd -= big_inc;
    else if (analogRead(pinKd) < (0.5 - zero_move_th) * 1024)
      val_inc_kd -= small_inc;
    else if (analogRead(pinKd) > (1 - big_move_th) * 1024)
      val_inc_kd += big_inc;
    else if (analogRead(pinKd) > (0.5 + zero_move_th) * 1024)
      val_inc_kd += small_inc;

    val_inc_kp = constrain(val_inc_kp, -1, 1);
    val_inc_ki = constrain(val_inc_ki, -1, 1);
    val_inc_kd = constrain(val_inc_kd, -1, 1);


    inc_kp = kp * pow(10, val_inc_kp);
    inc_kd = kd * pow(10, val_inc_kd);
    inc_ki = ki * pow(10, val_inc_ki);


    if (digitalRead(pinRSTKp) == LOW) {
      inc_kp = kp;
      val_inc_kp = 0;
    }
    if (digitalRead(pinRSTKd) == LOW) {
      inc_kd = kd;
      val_inc_kd = 0;
    }
    if (digitalRead(pinRSTKi) == LOW) {
      inc_ki = ki;
      val_inc_ki = 0;
    }


    loop_nr = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("kp:");
    lcd.print((round(inc_kp * 10) * 1.0 / 10));
    lcd.print(" ki:");
    lcd.print(round(inc_ki * 10) * 1.0 / 10);
    lcd.setCursor(0, 1);
    lcd.print("kd:");
    lcd.print(round(inc_kd * 10) * 1.0 / 10);
    lcd.print(" D:");
    lcd.print(round(dist) * 1.0 );
  }

  loop_nr++;

  pos_motor = PIDcalculation();
  // Send servo move command to write 0 degrees to servo ID 0x01
  //sendServoMoveCommand(0x01, pos_motor, 300/CRF);
  Sim();  // update Simulation; Simulated 'dist' is 'x1'; Simulated 'pos_motor' is 'u'
  Serial.print("dist: ");
  Serial.print/*ln*/(dist);
  Serial.print("\tx1: ");
  Serial.print(x1);
  Serial.print("\tpos_motor: ");
  Serial.print(pos_motor);
  Serial.print("\tu: ");
  Serial.println(u);

  servoCont.moveWithTime(1, pos_motor, 300/CRF);

  delay(1000 / CRF);
}

// Function to send data in the specified format
void sendData(byte data[], int length) {
  for (int i = 0; i < length; i++) {
    serial_motors.write(data[i]);
  }
}

// Function to create and send a command for servo movement
void sendServoMoveCommand(byte servoID, int angle, int time) {
  byte data[10];

  // Frame header
  data[0] = 0x55;
  data[1] = 0x55;

  // Calculate data length
  data[2] = 0x08;

  // Command byte
  data[3] = 0x03;  // CMD_SERVO_MOVE

  // Angle (high and low byte)
  data[4] = 0x01;         // Low byte
  data[5] = time & 0xFF;  // High byte

  // Set servo ID
  data[6] = (time >> 8) & 0xFF;
  data[7] = servoID;       //ID
  data[8] = angle & 0xFF;  // High byte of time

  data[9] = (angle >> 8) & 0xFF;

  // Send the data
  sendData(data, 10);
}

float PIDcalculation() {
  dist = sensor.getDistance()/*/10*/;
  error = setpoint - dist;

  changeError = error - last_error;                                             // derivative term
  totalError += error;                                                          //accumalate errors to find integral term
  totalError = constrain(totalError,-200,200);
  pidTerm = (-inc_kp * error) + (-inc_ki * totalError) + (-inc_kd * changeError);  //total gain
  //pidTerm = constrain(pidTerm, last_pidTerm - max_change_per_cycle, last_pidTerm + max_change_per_cycle);
  float y = 390 - pidTerm;//(int)constrain(pidTerm, -500, 500)+1500;  //constraining to appropriate value
  //y = pidTerm;
  last_error = error;
  last_pidTerm = pidTerm;
  return y;
}

float Sim(){
  error_Sim = setpoint - x1;
  i_error_Sim += error_Sim;
  i_error_Sim = constrain(i_error_Sim, -200, 200);
  d_error_Sim = error_Sim - last_error_Sim;
  last_error_Sim = error_Sim;
  u = (-inc_kp * error_Sim) + (-inc_ki * i_error_Sim) + (-inc_kd * d_error_Sim);
  x2 -= 9.80665 * u;
  x1 += x2;
}