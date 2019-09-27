/*
  SmoothTrajectory.h - Library to create a smooth transition from one point to another
  Created by Masoud Hassani, Sep 2019.
*/
#ifndef SmoothTrajectory_h
#define SmoothTrajectory_h

#include "Arduino.h"

class SmoothTrajectory
{
private:
    float p0;   // initial position
    float p1;   // final position
    float u;    // initial speed
    float v;    // final speed
    uint16_t T;    // transition period in milliseconds
    uint16_t t;   // current motion time in milliseconds
    uint32_t t0;  // initial time in milliseconds

public:
    // constructor
    SmoothTrajectory()
    {
        t0 = 0;
        t = 0;
        isMoving = false;
    }

    // public methods
    void reset(float pos0, float pos1, float sp0, float sp1, uint16_t per);
    float getPosition(uint8_t method);
    uint16_t getTimer();
    float position;
    bool  isMoving;  // a boolean showing if the the motion in trajectory is undergoing
};

#endif
