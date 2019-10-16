/*
  SmoothTrajectory.cpp - Library to create a smooth transition from one point to another
  Created by Masoud Hassani, Sep 2019.
*/

#include "Arduino.h"
#include "SmoothTrajectory.h"

float SmoothTrajectory::reset(float pos0, float pos1, float sp0, float sp1, uint16_t per, uint8_t m)
{
    p0 = pos0;  // mm
    p1 = pos1;  // mm
    u = sp0;    // mm/s
    v = sp1;    // mm/s
    T = per/1000.0;
    t0 = millis();
    method = m;

    if (method == 0){
        c3 = (2*(p0 - p1) + T*(u+v)) / pow(T,3);
        c2 = -1* (3*(p0 - p1) + 2*T*u + T*v) / pow(T,2);

        // calculate the max speed
        float tMax = -1.0*c2/(3.0*c3);  // time at which speed is max (by solving pdotdot = 0)
        maxSpeed = 3.0*c3*pow(tMax,2) + 2*c2*tMax + u;
    }
    return maxSpeed;
}

// smooth trajectory
// method=0: minimum acceleration
// method=1: minimum jerk
float SmoothTrajectory::getPosition()
{
    // calculate the current time with respect to the initialization of timer
    t = (millis() - t0)/1000.0;  // convert to seconds

    // if current time has passed the period
    if (t > T){
        t = T;
        isMoving = false;
    }

    if (method == 0){
        // calculate the next position
        position = c3*pow(t,3) + c2*pow(t,2) + u*t + p0;
    }
    return position;
}

// smooth trajectory
// method=0: minimum acceleration
// method=1: minimum jerk
float SmoothTrajectory::getSpeed()
{
    // calculate the current time with respect to the initialization of timer
    t = (millis() - t0)/1000.0;  // convert to seconds

    // if current time has passed the period
    if (t > T){
        t = T;
        isMoving = false;
    }

    if (method == 0){
        // calculate the next speed
        speed = 3.0*c3*pow(t,2) + 2*c2*t + u;
    }

    return speed;
}

float SmoothTrajectory::getTimer()
{
    return t;
}
