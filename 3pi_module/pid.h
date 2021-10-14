#ifndef _PID_H
#define _PID_H

#define MAX_OUTPUT 45

// Class to contain generic PID algorithm.
class PID_c {
  public:
  
    PID_c() {
    } 

    float Kp;                 // KpKi for wheel, KpKd for turn
    float Ki;
    float Kd;
    float e1, e2, e3;
    float output;

    void calculate_Increment(float input, float V_real)
    {
      e3 = e2; 
      e2 = e1;
      e1 = -V_real + input;

      output += Kp * (e1 - e2) +            // Increment PID Control
                Ki * (e1) + 
                Kd * (e1 - 2*e2 + e3);
    }
    
    void calculate_Position(float input, float V_real)
    {
      e3 = e2;
      e2 = e1;
      e1 = -V_real + input;

      output = Kp * e1 +                    // Position PID Control
               Ki * (e1 + e2 + e3) +        // approximation
               Kd * (e1 - e2);
    }
    

};



#endif
