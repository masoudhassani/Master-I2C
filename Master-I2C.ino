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
const uint8_t sizeOfData = 10;    // expected data from slave is 10 bytes. this might change
int8_t data[sizeOfData];
String command = "";

// master serial command variables
byte c;
String incommingData = "";
bool dir = false;

// controll that will controll all threads
ThreadController multiThread = ThreadController();

// initialize threads for multi tasking
Thread commanderThread = Thread();    // to send commands
Thread receiverThread = Thread();    // to receive data from motor

// function to read the master's serial bus and send the content to slave through i2c
void sendCommand()
{
    if (Serial.available() > 0){
        c = Serial.read();
        if (c != '\n'){
            incommingData += (char)c;
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
        int8_t setpointAngle = map(joyStick, 0, 1024, -45, 45);

        // print the angle setpoint from analog input
        Serial.print(setpointAngle);Serial.print('\t');

        char cstr[4];
        Wire.beginTransmission(driverAddress);
        Wire.write(itoa(setpointAngle, cstr, 10));
        Wire.endTransmission();
    }
}

void ReceiveData()
{
    uint8_t i = 0;
    Wire.requestFrom(driverAddress, 8);   // request 8 bytes from device
    while (Wire.available()) { // slave may send less than requested
        int8_t c = Wire.read(); // receive a byte as character
        data[i] = c;
        i += 1;
    }
    Serial.print(data[0]);Serial.print('\t');Serial.print(data[1]);Serial.print('\n');
}

void setup() {
    Wire.begin(0x00);        // join i2c bus
    Serial.begin(230400);    // setup serial for debug

    // setup threads
    commanderThread.onRun(sendCommand);
    commanderThread.setInterval(10);

    receiverThread.onRun(ReceiveData);
    receiverThread.setInterval(10);

    // Adds both threads to the controller
    multiThread.add(&commanderThread);
    multiThread.add(&receiverThread);

    if (debugMode){
        pinMode(analogInPin, INPUT);
    }
}


void loop()
{
    multiThread.run();
}
