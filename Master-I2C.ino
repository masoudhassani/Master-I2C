#include <Wire.h>
#include <Thread.h>
#include <ThreadController.h>
#include "Leg.h"

// ----------------------- main config of master code ---------------------
// define if the code is running in debug mode
bool  manualDriverTest = false;
bool  autoDriverTest = false;
bool  debugMode = true;
bool  showMotorData = true;

// ----------------------- i2c serial communication config --------------------
const uint8_t numDrivers = 3;
const uint8_t driverAddress[numDrivers] = { 0x01, 0x02, 0x03 };
const uint8_t masterAddress = 0x00;
const uint8_t sizeOfData = 4;    // expected data size from slave in number of bytes. this might change

// ----------------------- data structure for motor data ---------------------
/*
define data structure for receiving data
it creates a structure of 56 bites including one 32bit float,
one 16bit integer, one 8bit integer
*/
// data structure setup
typedef struct motorData_t
{
    float    angle;
    // float    velocity;
    // float    acceleration;
    // float    current;
    uint16_t    current;
    uint8_t  temperature;
    // uint8_t  booleans;
};

typedef union dataPackage_t
{
    motorData_t status;
    byte dataPackage[sizeof(motorData_t)];
};

dataPackage_t motor[numDrivers];

// ----------------------- robot geometry and definitions ---------------------
// leg foot needs to be redesigned. commented values are original
// for non-spherical leg foot
// current values assume a spherical foot but givez wrong height
float    L3 = 0.0;           // hip link length mm
float    L2 = 100.0;         // upper leg length mm
float    L1 = 155.0;         // lower leg length mm
float    X_OFFSET = -47.0;   // off set of foot contact point with ground with respect to hip joint
float    BODY_LENGTH = 298;  // length of the body from the front hip joint to the rear
float    BODY_WIDTH = 130;   // width of the body from left to right hip joint
float    Z0 = 220.0;         // initial height, based on neutral pos, 35 deg
float    legLength[3] = {L1, L2, L3};

// corner positions of the robot main body considering the CG as 0,0,0
float     cornerA[3] = {-1*BODY_LENGTH/2,-1*BODY_WIDTH/2, 0};
float     cornerB[3] = {-1*BODY_LENGTH/2, 1*BODY_WIDTH/2, 0};
float     cornerC[3] = {   BODY_LENGTH/2,-1*BODY_WIDTH/2, 0};
float     cornerD[3] = {   BODY_LENGTH/2, 1*BODY_WIDTH/2, 0};

// arrays containing euler angles of main body (RADIAN)
float     bodyAngle[3];    // roll (phi), pitch (theta), yaw (psi)

// arrays containing the limits of euler angles of body (DEGREE)
float     bodyAngleMin[3] = {-15.0, -15.0, -15.0};    // roll (phi), pitch (theta), yaw (psi)
float     bodyAngleMax[3] = { 15.0 , 15.0 , 15.0};    // roll (phi), pitch (theta), yaw (psi)

// initial configuration of a leg in terms of joint angle
float    jointNeutral[3] = {-57.6, 47.6, 0.0};    // neutral joint angle of each leg

// create leg instances
Leg leg[4] = {Leg("Rear Right", cornerA, legLength, X_OFFSET, Z0, jointNeutral),
    Leg("Rear Left", cornerB, legLength, X_OFFSET, Z0, jointNeutral),
    Leg("Front Right", cornerC, legLength, X_OFFSET, Z0, jointNeutral),
    Leg("Front Left", cornerD, legLength, X_OFFSET, Z0, jointNeutral)
};

// ----------------------- some decrelations ---------------------
// misc declerations
String incommingData = "";
int16_t setpointAngle;
uint16_t stepCounter = 0;
const int analogInPin = A0;

// ----------------------- multi threading stuff ---------------------
// controll that will controll all threads
ThreadController multiThread = ThreadController();

// initialize threads for multi tasking
Thread commanderThread = Thread();    // to send commands
Thread receiverThread = Thread();    // to receive data from motor
Thread motionThread = Thread();    // to move the motor to pre defined position

// function to read the master's serial bus and send the content to slave through i2c
void sendCommand()
{
    if (Serial.available() > 0){
        char c = Serial.read();
        if (c != '\n'){
            incommingData += c;
        }
        else{
            // interpret the incomming command and send it to drivers
            commandInterpreter(incommingData);

            // empty the buffer
            incommingData = "";
        }
    }

    // for debug use only
    // read analog input from potentiometer and map it to an angle range and send it to motor driver
    if (manualDriverTest){
        // read the analog in value
        int16_t joyStick = analogRead(analogInPin);

        // map it to the range of the angle out
        setpointAngle = map(joyStick, 0, 1024, -25, 25);

        // print the angle setpoint from analog input
        //Serial.print(setpointAngle);Serial.print('\t');

        char cstr[5];
        Wire.beginTransmission(driverAddress[0]);
        Wire.write(itoa(setpointAngle, cstr, 10));
        Wire.endTransmission();
    }
}

// function to receive motor driver data
void receiveData()
{
    for(int i=0; i<numDrivers; i++){
        byte *buffer;
        buffer = receive(driverAddress[i]);

        // recombine the separated 16bit data and assign it to corrent motor status variable
        // assign 8bit motor status variables
        int16_t angle = buffer[0];
        angle = angle << 8 | buffer[1];
        motor[i].status.current = buffer[2]*10;
        motor[i].status.temperature = buffer[3];

        // revert motor status data multipication to have the float number
        motor[i].status.angle = angle / 10.0;

    // show data in serial output
    }
    if (showMotorData){
        for(int i=0; i<numDrivers; i++){
            Serial.print(motor[i].status.angle);Serial.print('\t');
            Serial.print(motor[i].status.current);Serial.print('\t');
        }
        Serial.print('\n');
    }
    if (debugMode){
        Serial.print("X: ");Serial.print(leg[0].position[0]);Serial.print('\t');
        Serial.print("Y: ");Serial.print(leg[0].position[1]);Serial.print('\t');
        Serial.print("Z: ");Serial.println(leg[0].position[2]);
    }
}

// function to send automatic commands to move the motor
void motionControl()
{
    stepCounter += 1;
    if (stepCounter < 150){
        for(int i=0; i<numDrivers; i++){
            uint8_t flag = send(driverAddress[i], "G2A120");
        }
    }
    else if (stepCounter >= 150 and stepCounter < 300){
        for(int i=0; i<numDrivers; i++){
            uint8_t flag = send(driverAddress[i], "G2A0");
        }
    }
    else if (stepCounter >= 300 and stepCounter < 450){
        for(int i=0; i<numDrivers; i++){
            uint8_t flag = send(driverAddress[i], "G2A-120");
        }
    }
    else if (stepCounter >= 450 and stepCounter < 600){
        for(int i=0; i<numDrivers; i++){
            uint8_t flag = send(driverAddress[i], "G2A0");
        }
    }
    else{
        stepCounter = 0;
    }
}

void setup() {
    Wire.begin(0x00);        // join i2c bus
    Serial.begin(230400);    // setup serial for debug

    // setup threads
    commanderThread.onRun(sendCommand);
    commanderThread.setInterval(10);

    receiverThread.onRun(receiveData);
    receiverThread.setInterval(10);

    // Adds threads to the controller
    multiThread.add(&commanderThread);
    multiThread.add(&receiverThread);

    if (autoDriverTest){
        motionThread.onRun(motionControl);
        motionThread.setInterval(10);
        multiThread.add(&motionThread);
    }

    if (manualDriverTest){
        pinMode(analogInPin, INPUT);
    }

    Serial.println("Ready for commands");
    Serial.println("Use the dogzy command format, e.g. MxxGy");

    // assuming all joint angles are zero in the beginning
    // in the future a homing algorithm will make sure this assumption is correct
    for(int8_t i=0; i<4; ++i){
        leg[i].jointAngle[0] = jointNeutral[0];
        leg[i].jointAngle[1] = jointNeutral[1];
        leg[i].jointAngle[2] = jointNeutral[2];
    }

}


void loop()
{
    multiThread.run();
}
