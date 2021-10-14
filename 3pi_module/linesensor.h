#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define DN1_PIN 12      // ↓ from left to right
#define DN2_PIN A0
#define DN3_PIN A2
#define DN4_PIN A3
#define DN5_PIN A4
#define EMIT    11
#define LINE_VALUE 3100.0

class LineSensor_c {
  public:

    unsigned long startTime[5], endTime[5];
    unsigned long Fstd, last_us;
    int tag, dT[5], lineStandard[50], floorStandard[50];
    int ls_pin[5] = {DN1_PIN, DN2_PIN, DN3_PIN, DN4_PIN, DN5_PIN};
    float middle_line;
    bool detect_end[5];

    LineSensor_c() {
    }

    void initialise() {
      pinMode(DN1_PIN, INPUT);
      pinMode(DN2_PIN, INPUT);
      pinMode(DN3_PIN, INPUT);
      pinMode(DN4_PIN, INPUT);
      pinMode(DN5_PIN, INPUT);
      pinMode(EMIT, INPUT);
    }

    void detectLine() {
      // ************** charge the capacitor ***************
      pinMode(EMIT, OUTPUT);
      digitalWrite(EMIT, HIGH);   // open the IR only now so it won't influence RX and Serial function
      for (int i = 0; i < 5; i++)
      {
        pinMode(ls_pin[i], OUTPUT);
        digitalWrite(ls_pin[i], HIGH);
      }
      delayMicroseconds(10);
      for (int i = 0; i < 5; i++)
        pinMode(ls_pin[i], INPUT);

      // ******************  wait LOW  ******************
      for (int i = 0; i < 5; i++) {
        startTime[i] = micros();
        detect_end[i] = false;
      }
      last_us = micros();
      while (!detect_end[0] || !detect_end[1] || !detect_end[2] || !detect_end[3] || !detect_end[4])
      {
        for (int i = 0; i < 5; i++)
          if (digitalRead(ls_pin[i]) == LOW && !detect_end[i])
          {
            endTime[i] = micros();
            detect_end[i] = true;
          }
      }
      dT[0] = (int)(endTime[0] - startTime[0]);
      dT[1] = (int)(endTime[1] - startTime[1]);
      dT[2] = (int)(endTime[2] - startTime[2]);
      dT[3] = (int)(endTime[3] - startTime[3]);
      dT[4] = (int)(endTime[4] - startTime[4]);

      // ****************  calculate line position  ****************
      if (dT[2] > 5000)               // detact nothing
        middle_line = 10;
      else if (dT[0] > Fstd + 500)    // line on left
        middle_line = -1.8;
      else if (dT[4] > Fstd + 500)    // line on right
        middle_line = 1.8;
      else if (dT[2] > Fstd + 500)    // line on middle
      {
        if (dT[1] > dT[3])
          middle_line = -(float)(LINE_VALUE - dT[2]) / (float)(LINE_VALUE - Fstd);
        else
          middle_line = (float)(LINE_VALUE - dT[2]) / (float)(LINE_VALUE - Fstd);
      }
      else if (dT[1] > Fstd + 500)    // line on middle but left
        middle_line = -1.2;
      else if (dT[3] > Fstd + 500)    // line on middle but right
        middle_line = 1.2;
      else if (dT[2] < 5000)          // lose line but still on floor
        middle_line = 5;

      pinMode(EMIT, INPUT);
    }

};


#endif
