/*
 * File:          self_ballz.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define PI 3.1416

#define RADIUS 0.020
#define LENGTH 0.250

#define SERVO_MAX 180
#define SERVO_MIN 0
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  wb_keyboard_enable(1);
  int key =0;
  printf("Robot initialized\n");
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   WbDeviceTag tof = wb_robot_get_device("ToF Sensor");
   wb_distance_sensor_enable(tof, TIME_STEP);
   WbDeviceTag servo = wb_robot_get_device("ServoMotor");
   
 

   float x = LENGTH;
   float theta = 0;
   float xe = (LENGTH / 2) + 0.01;
   float e = 0;
   float e_ = 0; //previous error
   float int_e = 0; //integral of error
   float de_dt = 0; //derivative of error
   float kp = 30;
   float ki = 0.01;
   float kd = 90;
//   float edge_threshold = 0.05;
//   float edge_failsafe_percent = 0.4;
   float x_ = 0; //previous position
   float v = 0;
   float x_pred = LENGTH;
   float v_pred = 0;
   float theta_pred = 0;
   float e_pred_ = 0; //previous prediction error
   float int_e_pred = 0; //integral of prediction error
   float de_dt_pred = 0; //derivative of prediction error
   int start = 2;
   float dt = 0.039;
   float x_diff = 0;
   float v_diff = 0;
   int pred = 50;
   
   do{x = wb_distance_sensor_get_value(tof);} while(x == 1000);
   
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  printf("Robot control started\n");
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
     x = (wb_distance_sensor_get_value(tof) * LENGTH/1000) + RADIUS;
     if(start){
       x_pred = x;
       start--;
     }
     key = wb_keyboard_get_key();
     switch (key) {
       case 'A':
         kp = 1.1 * kp;
         break;
       case 'Z':
         kp = 0.9 * kp;
         break;
       case 'S':
         ki = 1.1 * ki;
         break;
       case 'X':
         ki = 0.9 * ki;
         break;
       case 'D':
         kd = 1.1 * kd;
         break;
       case 'C':
         kd = 0.9 * kd;
         break;
      }
       
     
  
    /* Process sensor data here */
     e = x - xe;
     int_e += e /** dt*/;
     de_dt = (e - e_) /*/ dt*/;
     e_ = e;
     theta = (kp * e + ki * int_e + kd * de_dt) * PI / 180;
     v = (x - x_) /*/dt*/;
     x_ = x;
/*     if(
       x > LENGTH - RADIUS - edge_threshold
       ||
       x < 0 + RADIUS + edge_threshold
     ) theta = theta * edge_failsafe_percent;
*/     
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     //printf("X:%.3f\ttheta:%.3f \tkp:%.3f\tki:%.3f\tkd:%.3f\n",x,theta,kp,ki,kd);
     if(!pred) wb_motor_set_position(servo, theta);
     
    // Prediction
     e = x_pred - xe;
     int_e_pred += e /** dt*/;
     de_dt_pred = (e - e_pred_) /*/ dt*/;
     e_pred_ = e;
     theta_pred = (kp * e + ki * int_e_pred + kd * de_dt_pred) * PI / 180;
     v_pred -= 9.80665 * theta_pred * dt;
     x_pred += v_pred * dt;
     
     if(pred){
       wb_motor_set_position(servo, theta_pred);
       pred--;
     }
     x_diff = x - x_pred;
     v_diff = v - v_pred;
     printf("X:%.3f\t\tvel:%.3f\t\ttheta:%.3f \tX_pred:%.3f\tvel_pred:%.3f\ttheta_pred:%.3f \tX_diff:%.3f\tvel_diff:%.3f\n",x,v,theta,x_pred,v_pred,theta_pred,x_diff,v_diff);
     
     //if(x_diff < -
  };
  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  //wb_robot_cleanup();

  return 0;
}
