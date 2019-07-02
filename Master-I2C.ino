#include <Wire.h>
#include <Thread.h>
#include <ThreadController.h>

// master and slave addresses
const int driverAddress = 0x01;
const int masterAddress = 0x00;

// i2c send/receive variables
const uint8_t sizeOfData = 8;    // expected data from slave is 8 bytes. this might change
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
            Wire.beginTransmission(driverAddress); // transmit to device #8
            Wire.write(command.c_str());              // sends one byte
            Wire.endTransmission();    // stop transmitting
            Serial.print("Echoing last command: ");Serial.print(command);Serial.print(" to slave 0x");Serial.println(driverAddress);
            incommingData = "";
        }
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
}


void loop()
{
    multiThread.run();
}
