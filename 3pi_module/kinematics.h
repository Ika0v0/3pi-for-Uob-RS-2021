#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "encoders.h"
#include "math.h"

#define wheel_L 96.0    // all mm / ms
#define wheel_R 16.0
#define wheel_cpr 358.3   // circle per revolution

class Kinematics_c {
  public:

    Kinematics_c() {
    }

    float position_X, position_Y;
    float theta;

    void Cal_position_frame(float vL, float vR) // based on global coordinate frame
    {
      float v_car = (vL + vR) / 2;
      float omega = (vR - vL) / wheel_L;
      position_X += v_car * cos(theta) * 20.0;
      position_Y += v_car * sin(theta) * 20.0;
      theta -= omega * 20.0;
    }

};



#endif
