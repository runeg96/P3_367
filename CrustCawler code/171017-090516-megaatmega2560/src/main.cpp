
#include <Arduino.h>
#include "Dynamixel_Serial.h"
#include <SoftwareSerial.h>

#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full control buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

float *data;

int generalVelocity = 1023;
int generalAcceleration = 32000;
//long previousTime1 = 0;
//float i = -80;

void setup(){

        Serial.flush();                                       // Clear the serial buffer of garbage data before running the code.
        Serial.begin(1000000);                                // Start serial communication on baudrate 1M
        Serial3.begin(1000000);                               // Start serial communication on 3rd serial on baudrate 1M
        Dynamixel.begin(1000000, 115200);                     // Initialise the Dynamixel class with baudrate 1M (Servos) and 115200 (XBee)

        Dynamixel.setDirectionPin(SERVO_ControlPin);          // Optional. Set direction control pin

        // // Turn on hold on the servos:
        Dynamixel.setHoldingTorque(0x01, true);               //Turn on hold torque on servo 1
        Dynamixel.setHoldingTorque(0x02, true);               //Turn on hold torque on servo 2
        Dynamixel.setHoldingTorque(0x03, true);               //Turn on hold torque on servo 3
        Dynamixel.setHoldingTorque(0x04, true);               //Turn on hold torque on servo 4
        Dynamixel.setHoldingTorque(0x05, true);               //Turn on hold torque on servo 5

        // Set the Profile velocities.
        Dynamixel.setProfileVelocity(0x01, generalVelocity); //Set the Profile Velocity for each servo. (max. is 1023)
        Dynamixel.setProfileVelocity(0x02, generalVelocity); //Set the Profile Velocity for each servo. (max. is 1023)
        Dynamixel.setProfileVelocity(0x03, generalVelocity); //Set the Profile Velocity for each servo. (max. is 1023)
        Dynamixel.setProfileVelocity(0x04, 500);             //Set the Profile Velocity for each servo. (max. is 1023)
        Dynamixel.setProfileVelocity(0x05, 500);             //Set the Profile Velocity for each servo. (max. is 1023)

        // Set the Profile accelerations.
        Dynamixel.setProfileAcceleration(0x01, generalAcceleration); //Set the Profile Acceleration for each servo. (max. is 32767)
        Dynamixel.setProfileAcceleration(0x02, generalAcceleration); //Set the Profile Acceleration for each servo. (max. is 32767)
        Dynamixel.setProfileAcceleration(0x03, generalAcceleration); //Set the Profile Acceleration for each servo. (max. is 32767)
        Dynamixel.setProfileAcceleration(0x04, 300);                 //Set the Profile Acceleration for each servo. (max. is 32767)
        Dynamixel.setProfileAcceleration(0x05, 300);                 //Set the Profile Acceleration for each servo. (max. is 32767)

         // Dynamixel.setGoalCurrent(0x02, 80);
         // Dynamixel.setGoalCurrent(0x03, 30);

         // Dynamixel.setOperationMode(0x01,3);
         // Dynamixel.setOperationMode(0x02,3);
         // Dynamixel.setOperationMode(0x03,3);
        // Calling the calibration method, to calibrate the EMG signals.
        //Dynamixel.Calibration();


 /*

        //Set baudrate for servos (3 = 1000000)
        Dynamixel.setBaudrate(0x01, 3);
        Dynamixel.setBaudrate(0x02, 3);
        Dynamixel.setBaudrate(0x03, 3);
        Dynamixel.setBaudrate(0x04, 3);
        Dynamixel.setBaudrate(0x05, 3);


        //Get position for servos
        Dynamixel.getPosition(0x01);
        Dynamixel.getPosition(0x02);
        Dynamixel.getPosition(0x03);
        Dynamixel.getPosition(0x04);
        Dynamixel.getPosition(0x05);

        //Get position from all servos at once
        Dynamixel.getPositionN();

        //Get position for servos in degrees
        Dynamixel.getPositionD(0x01);
        Dynamixel.getPositionD(0x02);
        Dynamixel.getPositionD(0x03);
        Dynamixel.getPositionD(0x04);
        Dynamixel.getPositionD(0x05);


        //Get load on servos in maximum procent
        Dynamixel.getLoad(0x02);

        //Kinematics tests
        Dynamixel.MoveL(0.362370,0.17774,0.27654,10);
        Dynamixel.setNGoalPositions(0,1573,1508,2174,2061);

        Dynamixel.getEE();
        Dynamixel.setEEPosition(0.2725745676, -0.2454272430, 0.3339601772, 1);
        Dynamixel.setNGoalPositions(1, 2048,2048,2048+250,2048-250);
        Dynamixel.setNGoalPositions(0, 1024, 2048, 2048, 2048);

*/
//Dynamixel.setNGoalPositions(0, 1048, 2048, 2048, 2048);
//Dynamixel.jointPlanner(0, 0, 0.549, 5);
for(float i = 0; i<5;i+=0.008){
Dynamixel.setGoalPosition(0x02, Dynamixel.positionOfTime(90, 180, i, 5));
//Dynamixel.setGoalPosition(0x03, Dynamixel.positionOfTime(180, 270, i, 5));
Dynamixel.getVelocity(0x02);
Serial.print("\t");
Dynamixel.getCurrent(0x02);
Serial.print("\t");
Dynamixel.getPositionD(0x02);
}
}


void loop(){
  // Dynamixel.setNGoalPositions(2048,2048,2048,2048,2048);
  // delay(1000);
  // Dynamixel.setNGoalPositions(2048,1024,2048,2048,2048);
  // delay(1000);
  Dynamixel.setHoldingTorque(0x01, true);               //Turn on hold torque on servo 1
  Dynamixel.setHoldingTorque(0x02, true);               //Turn on hold torque on servo 2
  Dynamixel.setHoldingTorque(0x03, true);               //Turn on hold torque on servo 3
  Dynamixel.setHoldingTorque(0x04, true);               //Turn on hold torque on servo 4
  Dynamixel.setHoldingTorque(0x05, true);               //Turn on hold torque on servo 5
  //Dynamixel.getPositionD(0x02);
          //long time1 = millis();
          //if(time1 - previousTime1 > 5000){
            //previousTime1 = time1;
          //   Dynamixel.getVelocity(0x01);
          //   Dynamixel.setGoalCurrent(0x01,i);
          //   i += 0.05;
          // //}
          // Serial.print(" Torque: ");
          // Serial.println(i);
          // delay(100);

        //Dynamixel.moveAll();
        //Dynamixel.getEE();
        //Dynamixel.readXbee();

        //Testing simple current control
        //Dynamixel.setGoalCurrent(0x01, 10);
        //Dynamixel.getCurrent(0x01);


        /*
        //Simple linear move test
        float endPosX;
        float endPosY;
        data = Dynamixel.getEE();

        endPosX = data[0];
        endPosY = data[1];

        endPosX +=0.01;
        endPosY +=0.01;

        Dynamixel.setEEPosition(endPosX, endPosY, 0.3, true);

        Serial.print("Calculated X: ");
        Serial.println(endPosX);
        Serial.print("Calculated X: ");
        Serial.println(endPosY);
        Serial.print("Measured Z: ");
        Serial.println(data[2]);
        */
}
