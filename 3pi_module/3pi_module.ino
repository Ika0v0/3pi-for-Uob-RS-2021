/*
   3pi+ Line-tracking car
   Designed for Robotics System Assignment 1
   Code by Minghua Li
   
   Sharing Version 1.0
   14/10/2021
*/

#include "motors.h"
#include "linesensor.h"
#include "bumpsensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

#define LED_PIN 13  // Yellow LED

Motors_c motors;
LineSensor_c linesensor;
Kinematics_c kinematics;
PID_c ML_pid, MR_pid, Line_pid;

int main_step, sub_step, index;
int loseLine;
volatile long count_e0_last;
volatile long count_e1_last;
float velocity_L, velocity_R;
float mm_per_revolution = 2.0 * 3.1416 * 16.0 / 358.3;
float v_straight, v_turn;
unsigned long last_time;

void setup() {      /****************Initial Setup******************/

  Serial.begin(9600);

  // Init Setup
  pinMode(LED_PIN, OUTPUT);
  delay(300);

  motors.initialise();
  linesensor.initialise();
  setupEncoder0();
  setupEncoder1();
  ML_pid.Kp = 1.0; ML_pid.Ki = 0.5; ML_pid.Kd = 0;          // change pid value by yourself
  MR_pid.Kp = 1.0; MR_pid.Ki = 0.5; MR_pid.Kd = 0;
  Line_pid.Kp = 1.0; Line_pid.Ki = 0; Line_pid.Kd = 0.5;
}

void control() {  /******************** FSM *******************/

  linesensor.detectLine();                                              // get line position
  velocity_R = (count_e0 - count_e0_last) * mm_per_revolution / 20.0;   // get right wheel speed (mm/ms)
  velocity_L = (count_e1 - count_e1_last) * mm_per_revolution / 20.0;   // get left wheel speed (mm/ms)
  count_e0_last = count_e0;
  count_e1_last = count_e1;

  kinematics.Cal_position_frame(velocity_L, velocity_R);                // update position

  if (main_step == 0)    // adjust floor IR value
    main_step = 1;
  else if (main_step == 1)    // start from home1, go straight till line
  {
    if (sub_step == 0) // go straight
    {
      ML_pid.calculate_Increment(5, velocity_L * 20.0 / mm_per_revolution);
      MR_pid.calculate_Increment(5, velocity_R * 20.0 / mm_per_revolution);
      if (linesensor.middle_line > 0 && linesensor.middle_line < 2)
        sub_step = 1;
    }
    else if (sub_step == 1) // turn left
    {
      ML_pid.calculate_Increment(-4, velocity_L * 20.0 / mm_per_revolution);
      MR_pid.calculate_Increment(4, velocity_R * 20.0 / mm_per_revolution);
      if (kinematics.theta < -1.56)
        main_step = 2;
    }
  }
  else if (main_step == 2)    // go to home2
  {
    if (sub_step == 1)
    {
      turn(linesensor.middle_line);
      if(loseLine > 100)
        sub_step = 2;
    }
    else if (sub_step == 2)    // lose line and turn 180°
    {
      ML_pid.calculate_Increment(4, velocity_L * 20.0 / mm_per_revolution);
      MR_pid.calculate_Increment(-2, velocity_R * 20.0 / mm_per_revolution);
      if (kinematics.theta > 1.56)
        sub_step = 3;
    }
    else if (sub_step == 3)
    {
      ML_pid.calculate_Increment(5, velocity_L * 20.0 / mm_per_revolution);
      MR_pid.calculate_Increment(5, velocity_R * 20.0 / mm_per_revolution);
      if (linesensor.middle_line < 2)
        sub_step = 4;
    }
    else if (sub_step == 4)           // follow line till the end
    {
      turn(linesensor.middle_line);
      if (loseLine > 100)
        sub_step = 5;
    }
    else if (sub_step == 5)           // then turn to home way
    {
      ML_pid.calculate_Increment(4, velocity_L * 20.0 / mm_per_revolution);
      MR_pid.calculate_Increment(-4, velocity_R * 20.0 / mm_per_revolution);
      if (atan2(kinematics.position_Y, kinematics.position_X) + 3.14 - kinematics.theta < 0.1)
        sub_step = 6;
    }
    else if (sub_step == 6)           // back home straightly
    {
      ML_pid.calculate_Increment(5, velocity_L * 20.0 / mm_per_revolution);
      MR_pid.calculate_Increment(5, velocity_R * 20.0 / mm_per_revolution);
      if (abs(kinematics.position_X) < 20 && abs(kinematics.position_Y) < 20)
        sub_step = 7;
    }
    else if (sub_step == 7)           // and stop
    {
      ML_pid.output = 0;
      MR_pid.output = 0;
    }
  }

  motors.set_Lspeed(ML_pid.output);
  motors.set_Rspeed(MR_pid.output);

}

void turn(float line)
{
  if (line > 4)                    // lose line
  {
    loseLine++;
    v_straight = 5;
    v_turn = 0;
    ML_pid.calculate_Increment(v_straight - v_turn, velocity_L * 20.0 / mm_per_revolution);
    MR_pid.calculate_Increment(v_straight + v_turn, velocity_R * 20.0 / mm_per_revolution);
  }
  else if (line < 2)                    // follow line
  {
    Line_pid.calculate_Position(0, line);
    v_straight = 5;
    v_turn = Line_pid.output * 5;
    loseLine = 0;
    ML_pid.calculate_Increment(v_straight - v_turn, velocity_L * 20.0 / mm_per_revolution);
    MR_pid.calculate_Increment(v_straight + v_turn, velocity_R * 20.0 / mm_per_revolution);
  }

}

void loop() {
  
    control();

    delay(19);
  
}
