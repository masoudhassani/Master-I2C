#include <Wire.h>
#include <Thread.h>
#include <ThreadController.h>

const int driverAddress = 0x01;
const int masterAddress = 0x00;
int8_t setpointAngle = 0;
int8_t data[8];
byte c;
String incommingData = "";
bool dir = false;

// controll that will controll all threads
ThreadController multiThread = ThreadController();

// initialize threads for multi tasking
Thread commanderThread = Thread();    // to send commands
Thread receiverThread = Thread();    // to receive data from motor

void sendCommand()
{
    if (Serial.available() > 0){
        c = Serial.read();
        if (c != '\n'){
            incommingData += (char)c;
        }
        else{
            setpointAngle = incommingData.toInt();
            incommingData = "";
        }
    }
    else{
        if (dir){
            setpointAngle += 15;
            if (setpointAngle > 14){
                dir = false;
            }
        }
        else{
            setpointAngle -= 15;
            if (setpointAngle < -14){
                dir = true;
            }
        }

    }
    //Serial.println(setpointAngle);

    Wire.beginTransmission(driverAddress); // transmit to device #8
    Wire.write(setpointAngle);              // sends one byte
    Wire.endTransmission();    // stop transmitting
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
    Serial.print(data[0]);Serial.print('\t');Serial.print(data[1]);Serial.print('\t');
    Serial.print('\n');
}

//StaticThreadController<2> controll (commanderThread, receiverThread);

void setup() {
    Wire.begin(0x00);        // join i2c bus
    Serial.begin(230400);    // setup serial for debug

    // setup threads
    commanderThread.onRun(sendCommand);
    commanderThread.setInterval(1000);

	receiverThread.onRun(ReceiveData);
	receiverThread.setInterval(10);

    // Adds both threads to the controller
	multiThread.add(&commanderThread);
	multiThread.add(&receiverThread);
}


void loop()
{
    multiThread.run();
}
