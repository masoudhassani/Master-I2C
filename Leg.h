/*
  Leg.h - Library for robot leg inverse kinematics.
  Created by Masoud Hassani, Feb 2019.
*/
#ifndef Leg_h
#define Leg_h

#include "Arduino.h"
class Leg
{
  public:
    Leg(String legName, float corner[3], float length[3], float x_offset, float height, float neutral[3]);
    void bodyRotToJointAngle(float th[3]);
    void coordinateToJointAngle();
    float position[3];
    float jointAngle[3];
    float jointNeutral[3];
    String name;
    float dx;
    float dy;
    float dz;
  private:
    float cX;
    float cY;
    float cZ;
    float L1;
    float L2;
    float L3;
    float xOffset;
    float Z0;
};

#endif
