/*
  leg.cpp - Library for robot leg inverse kinematics.
  Created by Masoud Hassani, Feb 2019.
*/

#include "Arduino.h"
#include "Leg.h"

Leg::Leg(String legName, float corner[3], float length[3], float x_offset, float height, float neutral[3])
{
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
    name        = legName;
    jointNeutral[0] = neutral[0];
    jointNeutral[1] = neutral[1];
    jointNeutral[2] = neutral[2];

    // initial offset (this is used to move the leg tip using serial commands)
    dx = 0.0;   // movement in mm in x direction
    dy = 0.0;   // movement in mm in y direction
    dz = 0.0;   // movement in mm in z direction
}

/*
function to calculate the leg tip position based on
specified pitch (theta), roll (phi) and yaw (psi) angle.
th is the body angle array.
this function first converts the body rotation to leg tip
then it calls the function coordinateToJointAngle to
convert the leg tip position to joint angle.
the origin of the body coordinate system is on the middle of body.
*/
void Leg::bodyRotToJointAngle(float th[3])
{
    float c1 = cos(th[0]);
    float c2 = cos(th[1]);
    float c3 = cos(th[2]);
    float s1 = sin(th[0]);
    float s2 = sin(th[1]);
    float s3 = sin(th[2]);
    position[0] = c2*c3*cX - c2*s3*cY + s2*cZ;
    position[1] = c1*s3*cX + c3*s1*s2*cX + c1*c3*cY - s1*s2*s3*cY - c2*s1*cZ;
    position[2] = s1*s3*cX - c1*c3*s2*cX + c3*s1*cY + c1*s2*s3*cY + c1*s2*cZ;

    // apply an offset based on the neutral position to assume the corner as reference (0,0,0)
    // if bodyRotation returns x=120 while x0=100, it means foot/ground should move -20
    position[0] = cX - position[0] + xOffset + dx;     // x is forward in the leg coordinate
    position[1] = L3 - (position[1] - cY) + dy;    // y is to right in the leg coordinate
    position[2] = Z0 + (position[2] - cZ) + dz;    // z is downward in the leg coordinate

    Leg::coordinateToJointAngle();
}

/*
function to calculate three joint angle of each leg from leg tip position
this function is called from bodyRotToJointAngle or can be called individually
*/
void Leg::coordinateToJointAngle()
{
    float theta[3];    // calculated joint angles in rad

    // inverse kinematics calculation
    //theta[2] = 2.0*atan((position[2]-sqrt(position[2]*position[2] + position[1]*position[1] - L3*L3)) / (L3 + position[1]));
    theta[2] = atan(position[1]/position[2]);  //t3
    float r1 = position[2] - L3 * sin(theta[2]);
    float r2 = position[0];
    float r3 = sqrt(r1*r1 + r2*r2);
    float psi2 = atan(r2 / r1);
    float phi2 = acos((L1*L1 - L2*L2 - r3*r3)/(-2*L2*r3));
    float phi1 = acos((r3*r3 - L1*L1 - L2*L2)/(-2*L2*L1));
    theta[1] = psi2 - phi2;   // t2
    theta[0] = PI - phi1;     // t1

    // convert to degree
    // angles are multiplied by -1 to correct the rotation direction
    jointAngle[2] = theta[2]*57.2958;
    jointAngle[1] = -1.0 * theta[1]*57.2958;
    jointAngle[0] = -1.0 * theta[0]*57.2958;
}
