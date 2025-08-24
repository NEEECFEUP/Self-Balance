/*

  SYSTEM

System starts when TOF detects the ball
  Systema follows control with feedback from ToF
    LED turns yellow at the ball's real position, read by ToF
    LED turns blue at the ball's simulated position
      LED becomes green when ball's real and simulated positions overlap!


  WIRING

Servo:
- Orange - PWM - [~3]
- Red - Vcc (4.8V-6V) - [5V]
- Brown - Ground - [GND]

Time-of-Flight VL53L0/1XV2 (ToF):
- Vin - Vcc (3.3V) - [3.3V]
- GND - Ground - [GND]
- SCL - Signal Clock - [SCL]
- SDA - Signal Data - [SDA]

*/

#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

#define R 20  // ball radius (ping pong ball) (in millimeters)
#define L 280   // gutter's length (in millimeters)

#define SERVO 3
#define SERVO_MAX 180 // input -> theta + 90°
#define SERVO_MIN 0

// z = tof_reading + R
#define Z_MAX L - R
#define Z_MIN R
/*
#define R wiringDigital
#define G wiringDigital
#define B wiringDigital

ver depois como funciona a strip que se escolher
*/
float
  z,  // output (position) (in millimeters)
  // x = (z , dz/dt)
  x1, // simulated z (position) (in millimeters)
  x2 = 0, // simulated dz/dt (in millimeters per second)
  theta = 90, // input (servo motor angle) (in degrees) // + 90° to ensure it stays in between 0° and 180°
  // u = theta
  u = 0,  // simulated theta (in degrees) // without + 90°
  ze = L/2, // z_ref = z_equilibrium (in millimeters)
  e,  // present error (in millimeters)
  e_, // previous error (in millimeters)
  ex_,  // previous simulated error (in millimeters)
  integral_e = 0, // error integral
  integral_ex = 0,  // simulated error integral
  de_dt,  // error derivative, derivative_e (de/dt)
  dex_dt, // simulated error derivative, derivative_ex (dex/dt)
  kp = 1,  // proportional constant
  ki = 0,  // integral constant
  kd = 0,  // derivative constant
  t,  // present time instant (in milliseconds)
  t_, // previous time instant (in milliseconds)
  dt  // time interval (t - t_) / 1000.0 (in seconds)
  ;

Servo motor;

VL53L0X tof;

void setup(){
// start Serial
  Serial.begin(9600);
// start I2C comms
  Wire.begin();
// set timeout for ToF
  tof.setTimeout(500);
// check proper detection
  if(!tof.init()){ // in case of no detection
    Serial.println("Error detecting VL53L0X!\nAborting...\n");  // print a warning
    while(1); // block further actions
  }
// set ToF sensor to continuous reading
  tof.startContinuous();
// attach Servo motor
  motor.attach(SERVO);
// initialize motor
  motor.write(round(theta));
// initialize system
  do{z = tof.readRangeContinuousMillimeters() + R;}while(z > Z_MAX);
// start system
  t_ = millis();  // get time instant and set as previous for convenience
  x1 = z;
}

void loop(){
  Serial.println(x1);
// make sure to note any ToF sensor timeout
  if(tof.timeoutOccurred()){
    Serial.println("VL53L0X TIMEOUT");
  }
// LEDs
/*
z = yellow
x1 = blue
*/
// time
  t = millis(); // get time instant
  dt = (t - t_) / 1000.0;
  t_ = t;
// position
  z = tof.readRangeContinuousMillimeters() + R;  // get ToF reading
// get theta
  e = z - ze; // update the error 
  integral_e += e * dt; // update the integral
  de_dt = (e - e_) / dt;  // update the derivative
  e_ = e; // update previous error
  theta = kp * e + ki * integral_e + kd * de_dt + 90; // calculate theta // + 90° to ensure it stays in between 0° and 180°
  motor.write(round(theta)); // feed the motor
// simulation
  e = x1 - ze;  // update the error
  integral_ex += e * dt;  // update the integral
  dex_dt = (e - ex_) / dt;  // update the derivative
  ex_ = e;  // update previous error
  u = kp * e + ki * integral_ex + kd * dex_dt;  // calculate angle
  x2 -= 9.80665 * u * dt; // update simulated speed
  x1 += x2 * dt;  // update simulated position
}