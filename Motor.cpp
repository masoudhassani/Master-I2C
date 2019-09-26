/*
  Motor.cpp - Library to communicate with motor drivers.
  Created by Masoud Hassani, Sep 2019.
*/
#include <Wire.h>
#include "Arduino.h"
#include "Motor.h"

void Motor::move(float desiredAngle)
{
    if (angle != desiredAngle){
        char command[10];
        uint8_t n = sprintf(command, "G2A%f", desiredAngle);
        send(address, command);
        isMoving = true;
    }
    else{
        isMoving = false;
    }
}

// function to update motor data such as angle, speed, acceleration
void Motor::update()
{
    byte *buffer;
    buffer = Motor::receive(address, sizeOfData);

    // recombine the separated 16bit data and assign it to corrent motor status variable
    // assign 8bit motor status variables
    int16_t angleBuffer = buffer[0];
    angleBuffer = angleBuffer << 8 | buffer[1];
    angle = angleBuffer / 10.0;
    current = buffer[2]*10;
    temperature = buffer[3];

    dt = micros() - timer;

    velocity = (angle - anglePrev) * 1000000 / dt;   // deg/sec
    anglePrev = angle;

    acceleration = (velocity - velocityPrev) * 1000000 / dt; //deg/s^2
    velocityPrev = velocity;
}

// function to send through i2c
uint8_t Motor::send(uint8_t addr, char * data)
{
    Wire.beginTransmission(addr);
    Wire.write(data);
    Wire.endTransmission();
    return 1;
}

// function to receive through i2c
// c++ does not allow the return of array. we sould use pointers
byte * Motor::receive(uint8_t addr, uint8_t size)
{
    uint8_t i = 0;
    static byte buff[4];

    Wire.requestFrom(addr, size);   // request sizeOfData bytes from device
    while (Wire.available()) {            // slave may send less than requested
        buff[i] = Wire.read();            // receive a byte
        i += 1;
    }

    return buff;
}

void Motor::setAddress(uint8_t addr)
{
    address = addr;
}
