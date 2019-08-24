#include <Wire.h>
#include <Thread.h>
#include <ThreadController.h>

// define if the code is running in debug mode
bool  debugMode = false;

// master and slave addresses
const int driverAddress = 0x01;
const int masterAddress = 0x00;
const int analogInPin   = A0;

// i2c send/receive variables
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

dataPackage_t motor;

// master serial command variables
String incommingData = "";
String command = "";
int16_t setpointAngle;
uint16_t stepCounter = 0;


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
            command = incommingData;
            Wire.beginTransmission(driverAddress);    // transmit to device
            Wire.write(command.c_str());              // sends a string
            Wire.endTransmission();                  // stop transmitting
            Serial.print("Echoing last command: ");Serial.print(command);Serial.print(" to slave 0x");Serial.println(driverAddress);
            incommingData = "";
        }
    }

    // for debug use only
    // read analog input from potentiometer and map it to an angle range and send it to motor driver
    if (debugMode){
        // read the analog in value
        int16_t joyStick = analogRead(analogInPin);

        // map it to the range of the angle out
        setpointAngle = map(joyStick, 0, 1024, -25, 25);

        // print the angle setpoint from analog input
        //Serial.print(setpointAngle);Serial.print('\t');

        char cstr[5];
        Wire.beginTransmission(driverAddress);
        Wire.write(itoa(setpointAngle, cstr, 10));
        Wire.endTransmission();
    }
}

// function to receive motor driver data
void receiveData()
{
    uint8_t i = 0;
    byte buffer[sizeOfData];

    Wire.requestFrom(driverAddress, sizeOfData);   // request sizeOfData bytes from device
    while (Wire.available()) {    // slave may send less than requested
        buffer[i] = Wire.read();  // receive a byte
        i += 1;
    }

    // recombine the separated 16bit data and assign it to corrent motor status variable
    // assign 8bit motor status variables
    int16_t angle = buffer[0];
    angle = angle << 8 | buffer[1];
    motor.status.current = buffer[2]*10;
    motor.status.temperature = buffer[3];

    // revert motor status data multipication to have the float number
    motor.status.angle = angle / 10.0;

    // read individual booleans from motor status boolean pack
    // bool rotatingCW = (motor.status.booleans) & 1;   // read the first bit
    // bool isAccelerating = (motor.status.booleans >> 1) & 1;   // read the second bit

    Serial.print(motor.status.angle);Serial.print('\t');
    //Serial.print(motor.status.current);Serial.print('\t');
    Serial.print('\n');
}

// function to send automatic commands to move the motor
void motionControl()
{
    stepCounter += 1;
    if (stepCounter < 150){
        Wire.beginTransmission(driverAddress);
        Wire.write("G2A120");          // set angle
        Wire.endTransmission();
    }
    else if (stepCounter >= 150 and stepCounter < 300){
        Wire.beginTransmission(driverAddress);
        Wire.write("G2A0");          // set angle
        Wire.endTransmission();
    }
    else if (stepCounter >= 300 and stepCounter < 450){
        Wire.beginTransmission(driverAddress);
        Wire.write("G2A-120");         // set angle
        Wire.endTransmission();
    }
    else if (stepCounter >= 450 and stepCounter < 600){
        Wire.beginTransmission(driverAddress);
        Wire.write("G2A0");         // set angle
        Wire.endTransmission();
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

    motionThread.onRun(motionControl);
    motionThread.setInterval(10);

    // Adds both threads to the controller
    multiThread.add(&commanderThread);
    multiThread.add(&receiverThread);
    multiThread.add(&motionThread);

    if (debugMode){
        pinMode(analogInPin, INPUT);
    }
}


void loop()
{
    multiThread.run();
}
