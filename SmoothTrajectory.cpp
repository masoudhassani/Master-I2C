/*
  SmoothTrajectory.cpp - Library to create a smooth transition from one point to another
  Created by Masoud Hassani, Sep 2019.
*/

#include "Arduino.h"
#include "SmoothTrajectory.h"

void SmoothTrajectory::reset(float pos0, float pos1, float sp0, float sp1, uint16_t per)
{
    p0 = pos0;
    p1 = pos1;
    u = sp0;
    v = sp1;
    T = per;
    t0 = millis();
}

// smooth trajectory
// method=0: minimum acceleration
// method=1: minimum jerk
float SmoothTrajectory::getPosition(uint8_t method)
{
    // calculate the current time with respect to the initialization of timer
    t = millis() - t0;

    // if current time has passed the period
    if (t > T){
        t = T;
        isMoving = false;
    }

    if (method == 0){
        float c3 = (2*(p0 - p1) + T*(u+v)) / pow(T,3);
        float c2 = -1* (3*(p0 - p1) + 2*T*u + T*v) / pow(T,2);
        position = c3*pow(t,3) + c2*pow(t,2) + u*t + p0;
    }
    return position;
}

uint16_t SmoothTrajectory::getTimer()
{
    return t;
}
