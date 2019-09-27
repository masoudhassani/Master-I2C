// return all joints to their neutral position
void neutral(){
    Serial.println("Neutral Position ...");
    bodyAngle[0] = 0.0; // initialize the roll angle of body
    bodyAngle[1] = 0.0; // initialize the pitch angle of body
    bodyAngle[2] = 0.0; // initialize the yaw angle of body
    for(int8_t i=0; i<4; ++i){
        leg[i].dx = 0.0;
        leg[i].dy = 0.0;
        leg[i].dz = 0.0;
        leg[i].bodyRotToJointAngle(bodyAngle);
    }
}

// return all joints to their zero position
void zero(){
    Serial.println("Zero Position ...");
    float p[3] = {X_OFFSET, 0.0, L1+L2-2};
    for(int8_t i=0; i<4; ++i){
        for(int j=0; j<3; j++){
            leg[i].waypoint[j] = p[j];
        }
    }
}

void moveLeg(uint8_t legNumber)
{
    for(int8_t i=0; i<3; ++i){
        char command[10];
        uint8_t n = sprintf(command, "G2A%f", leg[legNumber].jointAngle[i]);
        send(driverAddress[i], command);
    }
}
