#include <Wire.h>

const int driverAddress = 0x01;
const int masterAddress = 0x00;
int8_t setpointAngle = 0;
byte c;
String incommingData = "";

void setup() {
  Wire.begin(0x00); // join i2c bus (address optional for master)
  Serial.begin(230400);
}


void loop() {
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
    //Serial.println(setpointAngle);
    Wire.requestFrom(driverAddress, 8);   // request 8 bytes from device
    while (Wire.available()) { // slave may send less than requested
        uint8_t c = Wire.read(); // receive a byte as character
        Serial.print(c);Serial.print('\t');        // print the character
    }
    Serial.print('\n');
    Wire.beginTransmission(driverAddress); // transmit to device #8
    Wire.write(setpointAngle);              // sends one byte
    Wire.endTransmission();    // stop transmitting
    delay(10);
}
