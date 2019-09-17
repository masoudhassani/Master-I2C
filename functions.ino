// function to send through i2c
uint8_t send(uint8_t addr, char * data)
{
    Wire.beginTransmission(addr);
    Wire.write(data);
    Wire.endTransmission();
    return 1;
}

// function to receive through i2c
// c++ does not allow the return of array. we sould use pointers
byte * receive(uint8_t addr)
{
    uint8_t i = 0;
    static byte buff[sizeOfData];

    Wire.requestFrom(addr, sizeOfData);   // request sizeOfData bytes from device
    while (Wire.available()) {            // slave may send less than requested
        buff[i] = Wire.read();            // receive a byte
        i += 1;
    }

    return buff;
}

// interpreter function to convert serial or hardcoded commands to
// something meaningful for each motor driver
void commandInterpreter(String strCommand)
{
    uint8_t cmdLength = strCommand.length();
    String motorCommand;
    String legCommand;
    int8_t ind;
    uint8_t intLength = 2;   // number of motors can be double digit

    // interpret commands starting with 'm' or 'M' to command individual motors
    if(strCommand.charAt(0)=='M' || strCommand.charAt(0)=='m'){
        // read the integer after the command definition. intLength digit(s) int is expected
        uint8_t motorNumber = strCommand.substring(1,intLength+1).toInt();

        // extract the motor command
        motorCommand =  strCommand.substring(intLength,cmdLength);

        // send the command
        uint8_t flag = send(driverAddress[motorNumber], const_cast<char*> (motorCommand.c_str()));

        // print received command if debug mode
        if (debugMode){
            Serial.print("Echoing last command: ");Serial.print(motorCommand);
            Serial.print(" to slave 0x");Serial.println(driverAddress[motorNumber]);
        }
    }

    // commands for all motors
    else if (strCommand.charAt(0)=='C' || strCommand.charAt(0)=='c'){
        // read the integer after the command definition. intLength digit(s) int is expected
        uint8_t command = strCommand.substring(1,intLength+1).toInt();

        switch (command) {
            case 00:
                neutral();
                break;
            case 99:
                for(int i=0; i<numDrivers; i++){
                    uint8_t flag = send(driverAddress[i], "G2A0");
                }
                break;
            case 88:
                for(int i=0; i<numDrivers; i++){
                    uint8_t flag = send(driverAddress[i], "G2A50");
                }
                break;
        }
    }

    // interpret commands starting with 'm' or 'M' to command individual legs
    else if(strCommand.charAt(0)=='L' || strCommand.charAt(0)=='l'){
        // read the integer after the command definition. 1 digit int is expected
        uint8_t legNumber = strCommand.substring(1,2).toInt();

        // extract the motor command
        legCommand =  strCommand.substring(2,cmdLength);
        ind = legCommand.length();

        if (legCommand.charAt(0)=='X' || legCommand.charAt(0)=='x'){
            leg[legNumber].position[0] = legCommand.substring(1,ind).toFloat();
        }

        else if (legCommand.charAt(0)=='Y' || legCommand.charAt(0)=='y'){
            leg[legNumber].position[1] = legCommand.substring(1,ind).toFloat();
        }

        else if (legCommand.charAt(0)=='Z' || legCommand.charAt(0)=='z'){
            leg[legNumber].position[2] = legCommand.substring(1,ind).toFloat();
        }

        // update the joint angles of the requested leg
        leg[legNumber].coordinateToJointAngle();

        // move the requested leg
        moveLeg(legNumber);

        // print received command if debug mode
        if (debugMode){
            Serial.print("Echoing last command: ");Serial.print(legCommand);
            Serial.print(" to leg ");Serial.println(leg[legNumber].name);
        }
    }

    // in case incomming command has unknown format
    else{
        if (debugMode){
            Serial.println("Command could not be interpreted!");
        }
    }
}
