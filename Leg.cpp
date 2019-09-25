/*
  leg.cpp - Library for robot leg inverse kinematics.
  Created by Masoud Hassani, Feb 2019.
*/

#include "Arduino.h"
#include "Leg.h"
#include "Motor.h"

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
    // desired position array
    float p[3];

    float c1 = cos(th[0]);
    float c2 = cos(th[1]);
    float c3 = cos(th[2]);
    float s1 = sin(th[0]);
    float s2 = sin(th[1]);
    float s3 = sin(th[2]);
    p[0] = c2*c3*cX - c2*s3*cY + s2*cZ;
    p[1] = c1*s3*cX + c3*s1*s2*cX + c1*c3*cY - s1*s2*s3*cY - c2*s1*cZ;
    p[2] = s1*s3*cX - c1*c3*s2*cX + c3*s1*cY + c1*s2*s3*cY + c1*s2*cZ;

    // apply an offset based on the neutral position to assume the corner as reference (0,0,0)
    // if bodyRotation returns x=120 while x0=100, it means foot/ground should move -20
    p[0] = cX - p[0] + xOffset + dx;     // x is forward in the leg coordinate
    p[1] = L3 - (p[1] - cY) + dy;    // y is to right in the leg coordinate
    p[2] = Z0 + (p[2] - cZ) + dz;    // z is downward in the leg coordinate

    // send out the desired position to another function to actuate the motors
    Leg::coordinateToJointAngle(p);
}

/*
function to calculate three joint angle of each leg from leg tip position
this function is called from bodyRotToJointAngle or can be called individually
*/
void Leg::coordinateToJointAngle(float p[3])
{
    if (p[0] != position[0] || p[1] != position[1] || p[2] != position[2]){
        float theta[3];    // calculated joint angles in rad

        // inverse kinematics calculation
        //theta[2] = 2.0*atan((position[2]-sqrt(position[2]*position[2] + position[1]*position[1] - L3*L3)) / (L3 + position[1]));
        theta[2] = atan(p[1]/p[2]);  //t3
        float r1 = p[2] - L3 * sin(theta[2]);
        float r2 = p[0];
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

        // actuate all joints
        Leg::moveLeg();

        // update current position, this might be unnecessary
        position[0] = p[0];
        position[1] = p[1];
        position[2] = p[2];
    }
}

//forward kinematics, angle to x,y,z
// angle is in rad and position is in mm
void Leg::jointAngleToCoordinate(float th[3])
{
    position[0] = L1*sin((th[0]+th[1])/57.2958) + L2*sin(th[1]/57.2958);
    position[1] = (L1*cos((th[0]+th[1])/57.2958) + L2*cos(th[1]/57.2958))*sin(th[2]/57.2958);
    position[2] = (L1*cos((th[0]+th[1])/57.2958) + L2*cos(th[1]/57.2958))*cos(th[2]/57.2958);
}

// move a leg to desired position by moving all joints
void Leg::moveLeg()
{
    for(int i=0; i<3; i++){
        joint[i].move(jointAngle[i]);
    }
}

// update the motor data and leg position
void Leg::update()
{
    float th[3] = {-1.0*joint[0].angle, -1.0*joint[1].angle, joint[2].angle};

    for(int i=0; i<3; i++){
        joint[i].update();
    }

    Serial.print(th[0]); Serial.print('\t');
    Serial.print(th[1]); Serial.print('\t');
    Serial.println(th[2]);

    Leg::jointAngleToCoordinate(th);
}
