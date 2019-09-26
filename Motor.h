/*
  Motor.h - Library to communicate with motor drivers.
  Created by Masoud Hassani, Sep 2019.
*/
#ifndef Motor_h
#define Motor_h

#include <Wire.h>
#include "Arduino.h"
class Motor
{
private:
    float initialAngle;
    uint32_t timer;
    uint16_t dt;
    float anglePrev;
    float velocityPrev = 0.0;
    const uint8_t sizeOfData;
    uint8_t address;

  public:
    // constructor
    Motor(uint8_t addr = 0x01, float init = 0.0, uint8_t dataSize = 4):
    sizeOfData(dataSize)
    {
        address = addr;
        initialAngle = init;
    }

    // public methods
    void move(float desiredAngle);
    void update();
    uint8_t send(uint8_t addr, char * data);
    byte * receive(uint8_t addr, uint8_t size);
    void setAddress(uint8_t addr);

    // public member variables
    float angle = initialAngle;
    float velocity = 0.0;
    float acceleration = 0.0;
    uint8_t temperature = 0.0;
    uint16_t current = 0.0;
    bool isMoving = false;
};

#endif
