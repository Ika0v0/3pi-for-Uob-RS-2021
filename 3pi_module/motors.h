#ifndef _MOTORS_H
#define _MOTORS_H

class Motors_c {
  public:
  #define MaxSpeed 100
  #define L_DIR_PIN 16
  #define R_DIR_PIN 15
  #define L_PWM_PIN 10
  #define R_PWM_PIN 9
  
  Motors_c() {
  } 

  void initialise() {
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(R_PWM_PIN, OUTPUT);
  }

  void set_Lspeed(int v){    // input range from -MaxSpeed to MaxSpeed
    if(v >= 0 && v <= MaxSpeed)
    {
      digitalWrite(L_DIR_PIN, LOW);
      analogWrite(L_PWM_PIN, v);
    }
    else if(v < 0 && v >= -MaxSpeed)
    {
      digitalWrite(L_DIR_PIN, HIGH);
      analogWrite(L_PWM_PIN, -v);
    }
  }
  void set_Rspeed(int v){     // same as L
    if(v >= 0 && v <= MaxSpeed)
    {
      digitalWrite(R_DIR_PIN, LOW);
      analogWrite(R_PWM_PIN, v);
    }
    else if(v < 0 && v >= -MaxSpeed)
    {
      digitalWrite(R_DIR_PIN, HIGH);
      analogWrite(R_PWM_PIN, -v);
    }
  }
    
};

#endif
