/*
  Leg.h - Library for robot leg inverse kinematics.
  Created by Masoud Hassani, Feb 2019.
*/
#ifndef Leg_h
#define Leg_h

#include "Arduino.h"
#include "Motor.h"
class Leg
{
private:
    void moveLeg();
    float cX;
    float cY;
    float cZ;
    float L1;
    float L2;
    float L3;
    float xOffset;
    float Z0;
    uint8_t address[3];
    Motor joint[3];

public:
    Leg(String legName, float corner[3], float length[3], float x_offset, float height, float neutral[3], uint8_t addr[3])

    {
        address[0] = addr[0]; address[1] = addr[1]; address[2] = addr[2];

        // initial corner positions of body which are initial leg root position
        cX = corner[0];
        cY = corner[1];
        cZ = corner[2];

        // initialize the leg geometry
        L1 = length[0];
        L2 = length[1];
        L3 = length[2];
        xOffset = x_offset;
        Z0 = height;

        // leg foot/ground interface (leg tip) position with respect to its root
        position[0] = corner[0];
        position[1] = corner[1];
        position[2] = corner[2];

        name = legName;
        jointNeutral[0] = neutral[0];
        jointNeutral[1] = neutral[1];
        jointNeutral[2] = neutral[2];

        // initial offset (this is used to move the leg tip using serial commands)
        dx = 0.0;   // movement in mm in x direction
        dy = 0.0;   // movement in mm in y direction
        dz = 0.0;   // movement in mm in z direction

        // set joint motor address
        for(int i=0; i<3; i++){
            joint[i].setAddress(address[i]);
        }
    }

    void bodyRotToJointAngle(float th[3]);
    void coordinateToJointAngle(float p[3]);
    float position[3];
    float jointAngle[3];
    float jointNeutral[3];
    String name;
    float dx;
    float dy;
    float dz;
};

#endif
