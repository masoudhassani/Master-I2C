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
    float       p0;   // initial position
    float       p1;   // final position
    float       u;    // initial speed
    float       v;    // final speed
    float       T;    // transition period in seconds
    float       t;    // current motion time in seconds
    uint32_t    t0;  // initial time in milliseconds
    float       c3;
    float       c2;

public:
    // constructor
    SmoothTrajectory()
    {
        t0 = 0;
        t = 0;
        isMoving = false;
        speed = 0.0;
    }

    // public methods
    float   reset(float pos0, float pos1, float sp0, float sp1, uint16_t per, uint8_t m);
    float   getPosition();
    float   getSpeed();
    float   getTimer();
    float   position;
    float   speed;
    float   maxSpeed;
    float   scale;
    bool    isMoving;  // a boolean showing if the the motion in trajectory is undergoing
    uint8_t method;
};

#endif
