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

    // ------------------------------------------------------------------------
    // interpret commands starting with 'm' or 'M' to command individual motors
    // command is in form of Mxx[motor command] e.g. M2G2A120 to move the 3rd
    // motor to 120 deg
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

    // ------------------------------------------------------------------------
    // commands for all motors at the same time
    // c00: all motors to neutral position
    // c99: all motors to 0 deg
    // c88: all motors to 20 deg
    else if (strCommand.charAt(0)=='C' || strCommand.charAt(0)=='c'){
        // read the integer after the command definition. intLength digit(s) int is expected
        uint8_t command = strCommand.substring(1,intLength+1).toInt();

        switch (command) {
            case 99:
                neutral();
                break;

            case 00:
                zero();
                break;

            case 88:
                for(int i=0; i<numJoints; i++){
                    uint8_t flag = send(driverAddress[i], "G2A20");
                }
                break;
        }
    }

    // interpret commands starting with 'l' or 'L' to command individual legs
    // command is in form of Lx[leg command]:
    // LxX100: moved the xth leg's tip 100mm in X direction
    // LxY100: moved the xth leg's tip 100mm in Y direction
    // LxZ100: moved the xth leg's tip 100mm in Z direction
    else if(strCommand.charAt(0)=='L' || strCommand.charAt(0)=='l'){
        // read the integer after the command definition. 1 digit int is expected
        uint8_t legNumber = strCommand.substring(1,2).toInt();

        // extract the motor command
        legCommand =  strCommand.substring(2,cmdLength);
        ind = legCommand.length();

        // create a list to hold requested leg position
        float p[3] = {leg[legNumber].position[0], leg[legNumber].position[1], leg[legNumber].position[2]};

        if (legCommand.charAt(0)=='X' || legCommand.charAt(0)=='x'){
            p[0] = legCommand.substring(1,ind).toFloat();
        }

        else if (legCommand.charAt(0)=='Y' || legCommand.charAt(0)=='y'){
            p[1] = legCommand.substring(1,ind).toFloat();
        }

        else if (legCommand.charAt(0)=='Z' || legCommand.charAt(0)=='z'){
            p[2] = legCommand.substring(1,ind).toFloat();
        }

        // update the joint angles of the requested leg
        leg[legNumber].coordinateToJointAngle(p);

        // move the requested leg
        //moveLeg(legNumber);

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
