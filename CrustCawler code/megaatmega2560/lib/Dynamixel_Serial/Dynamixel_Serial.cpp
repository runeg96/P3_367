#include "Dynamixel_Serial.h"
#include "Wire.h"
#include "math.h"
#include "StandardCplusplus.h"
#include "vector"


//##############################################################################
//############################ Public Methods ##################################
//##############################################################################


void DynamixelClass::begin(long baud, long baud2){

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
        Serial1.begin(baud); // Set up Serial for Leonardo and Mega
        Serial2.begin(baud2);
        _serial = &Serial1;
        _serial2 = &Serial2;


#else
        Serial.begin(baud); // Set up Serial for all others (Uno, etc)
        _serial = &Serial;
#endif

}

void DynamixelClass::begin(HardwareSerial &HWserial, long baud){

        HWserial.begin(baud); // Set up Serial for a specified Serial object
        _serial = &HWserial;

}

void DynamixelClass::begin(Stream &serial){

        _serial = &serial; // Set a reference to a specified Stream object (Hard or Soft Serial)

}

void DynamixelClass::end(){

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
        Serial1.end();
#else
        Serial.end();
#endif

}


//      ███████╗███████╗████████╗████████╗███████╗██████╗ ███████╗
//     ██╔════╝██╔════╝╚══██╔══╝╚══██╔══╝██╔════╝██╔══██╗██╔════╝
//    ███████╗█████╗     ██║      ██║   █████╗  ██████╔╝███████╗
//   ╚════██║██╔══╝     ██║      ██║   ██╔══╝  ██╔══██╗╚════██║
//  ███████║███████╗   ██║      ██║   ███████╗██║  ██║███████║
//  ╚══════╝╚══════╝   ╚═╝      ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝


void DynamixelClass::setDirectionPin(unsigned char D_Pin){

        Direction_Pin = D_Pin;
        pinMode(Direction_Pin,OUTPUT);

}

void DynamixelClass::setHoldingTorque(unsigned char ID, bool Set) {
        unsigned char arr[1] = {Set};
        writeN(ID, 0x40, arr, 1);
}

void DynamixelClass::setOperationMode(unsigned char ID, unsigned int mode){
        unsigned char arr[1]={(mode & 0xFF)};
        writeN(ID,0x0B,arr,1);
        clearRXbuffer();
        readN(ID,0x0B,4);
        getParameters();

        int sum;
        sum = data[1]; //Converting two information bytes into a integer (position data)

        //Debug information
        Serial.print("OperationMode of ID: ");
        Serial.print(data[0]);
        Serial.print(" is ");
        Serial.println(sum);
}

void DynamixelClass::setBaudrate(unsigned char ID, unsigned int baud){
        unsigned char arr[1]={(baud & 0xFF)};
        writeN(ID,0x08,arr,1);
}

void DynamixelClass::setDriveMode(unsigned char ID, bool direction){
        unsigned char arr[1]={direction};
        writeN(ID,0x0A,arr,1);
}

void DynamixelClass::setGoalPosition(unsigned char ID, unsigned int pos) {

        pos %= 4096;

        switch (ID) {
        case 0x02:
        case 0x03:
                if(pos < 760) {
                        pos = 760;
                }
                if(pos > 3320) {
                        pos = 3320;
                }
                break;
        case 0x04:
                if(pos < 1940) {
                        pos = 1940;
                }
                if(pos >3320) {
                        pos = 3320;
                }
                break;
        case 0x05:
                if(pos < 760) {
                        pos = 760;
                }
                if(pos >2145) {
                        pos = 2145;
                }
                break;
        }


        unsigned char arr[] = {
                (pos & 0xFF),
                (pos & 0xFF00) >> 8,
                (pos & 0xFF0000) >> 16,
                (pos & 0xFF000000) >> 24
        };

        writeN(ID, 0x74, arr, 4);
}

void DynamixelClass::setGoalCurrent(unsigned char ID, unsigned int current) {

        unsigned char arr[] = {
                (current & 0xFF),
                (current & 0xFF00) >> 8,
                (current & 0xFF0000) >> 16,
                (current & 0xFF000000) >> 24
        };

        writeN(ID, 0x66, arr, 2);
}

void DynamixelClass::setNGoalPositions(int m1, int m2, int m3, int m4, int m5) {

        int n = 0;
        int arr[5] = {m1, m2, m3, m4, m5};
        for (int i = 0; i < 5; i++) {
                if(arr[i] > -1)
                        n++;
        }
        unsigned char *pt = new unsigned char[n*5];
        int nn = 0;

        for (int i = 0; i < 5; i++) {
                if(arr[i] > -1) {
                        pt[nn*5] = i+1;
                        pt[nn*5+1] = (arr[i] & 0xFF);
                        pt[nn*5+2] = (arr[i] & 0xFF00) >> 8;
                        pt[nn*5+3] = (arr[i] & 0xFF0000) >> 16;
                        pt[nn*5+4] = (arr[i] & 0xFF000000) >> 24;
                        nn++;
                }
        }
        syncWN(0x74, pt, nn*5, 4);
        //delete pt;
}

void DynamixelClass::setProfileAcceleration(unsigned char ID, unsigned int pac) {

        pac %= 32767;

        unsigned char arr[] = {
                (pac & 0xFF),
                (pac & 0xFF00) >> 8,
                (pac & 0xFF0000) >> 16,
                (pac & 0xFF000000) >> 24
        };

        writeN(ID, 0x6C, arr, 4);
}

void DynamixelClass::setProfileVelocity(unsigned char ID, unsigned int pvl) {

        pvl %= 1023;

        unsigned char arr[] = {
                (pvl & 0xFF),
                (pvl & 0xFF00) >> 8,
                (pvl & 0xFF0000) >> 16,
                (pvl & 0xFF000000) >> 24
        };

        writeN(ID, 0x70, arr, 4);
}

void DynamixelClass::setGoalVelocity(unsigned char ID, unsigned int vel){

        vel  %= 1023;

        unsigned char arr[] = {
                (vel & 0xFF),
                (vel & 0xFF00) >> 8,
                (vel & 0xFF0000) >> 16,
                (vel & 0xFF000000) >> 24
        };

        writeN(ID, 0x68, arr, 4);
}

void DynamixelClass::setLimits(unsigned char ID, int max, int min){
        max  %= 4095;

        unsigned char arr[] = {
                (max & 0xFF),
                (max & 0xFF00) >> 8,
                (max & 0xFF0000) >> 16,
                (max & 0xFF000000) >> 24
        };

        writeN(ID, 0x30, arr, 4);

        min  %= 4095;

        unsigned char arr1[] = {
                (min & 0xFF),
                (min & 0xFF00) >> 8,
                (min & 0xFF0000) >> 16,
                (min & 0xFF000000) >> 24
        };

        writeN(ID, 0x34, arr1, 4);

}

void DynamixelClass::setEEPosition(double x, double y, double z, bool move){

        //Calculations for Inverse Kinematicsc
        double x0 = sqrt((x * x) + (y * y));
        z -= 0.06;
        float a1 = 0.22;
        float a2 = 0.27;

        double r = sqrt((x0*x0)+(z*z));

        double theta1 =  atan2(y, x)*(180/M_PI);
        if (theta1 < 0) {
                theta1 += 360;
        }

        double theta2Calc = acos(((a1 * a1) - (a2 * a2) + (r * r)) / (2 * a1 * r)); //Second argument for the calculation
        double theta2p =  (atan2(z, x0) + theta2Calc) * (180 / M_PI) + 90;
        double theta2n =  (atan2(z, x0) - theta2Calc) * (180 / M_PI) + 90;

        double theta3Calc = acos(((a1 * a1) + (a2 * a2) - (r * r)) / (2 * a1 * a2)) - M_PI; //Second argument for the calculation
        double theta3p =  (M_PI + theta3Calc) * 180 / M_PI;
        double theta3n =  (M_PI - theta3Calc) * 180 / M_PI;

        //Calculating difference, to find closest position
        double diff2p = 0, diff2n = 0, diff3p = 0, diff3n = 0;

        diff2p = fabs(degToPos(theta2p) - getPosition(0x02));
        diff3p = fabs(degToPos(theta3p) - getPosition(0x03));
        diff2n = fabs(degToPos(theta2n) - getPosition(0x02));
        diff3n = fabs(degToPos(theta3n) - getPosition(0x03));

        //Error protection
        if(theta1 != theta1 || theta2p != theta2p || theta3p != theta3p || theta2n != theta2n || theta3n != theta3n)
        {
                Serial.println("NaN Error");
        }
        else
        {
                //Checking nearest position, and if it's within the angle limits
                if (degToPos(theta2p) > 760 && degToPos(theta2p) < 3340
                    && degToPos(theta3p) > 760 && degToPos(theta3p) < 3335) {

                        if (degToPos(theta2n) > 760 && degToPos(theta2n) < 3340
                            && degToPos(theta3n) > 760 && degToPos(theta3n) < 3335) {

                                if (diff2p + diff3p < diff2n + diff3n) {
                                        if(move) {
                                                setGoalPosition(0x01,degToPos(theta1));
                                                setGoalPosition(0x02,degToPos(theta2p));
                                                setGoalPosition(0x03,degToPos(theta3p));
                                        }else{
                                                returnPos[0]=theta1;
                                                returnPos[1]=theta2p;
                                                returnPos[2]=theta3p;
                                        }

                                } else {
                                        if(move) {
                                                setGoalPosition(0x01,degToPos(theta1));
                                                setGoalPosition(0x02,degToPos(theta2n));
                                                setGoalPosition(0x03,degToPos(theta3n));
                                        }else{
                                                returnPos[0]=theta1;
                                                returnPos[1]=theta2n;
                                                returnPos[2]=theta3n;
                                        }
                                }
                        } else {
                                if(move) {
                                        setGoalPosition(0x01,degToPos(theta1));
                                        setGoalPosition(0x02,degToPos(theta2p));
                                        setGoalPosition(0x03,degToPos(theta3p));
                                }else{
                                        returnPos[0]=theta1;
                                        returnPos[1]=theta2p;
                                        returnPos[2]=theta3p;
                                }

                        }
                } else if(degToPos(theta2n) > 760 && degToPos(theta2n) < 3340
                          && degToPos(theta3n) > 760 && degToPos(theta3n) < 3335) {
                        if(move) {
                                setGoalPosition(0x01,degToPos(theta1));
                                setGoalPosition(0x02,degToPos(theta2n));
                                setGoalPosition(0x03,degToPos(theta3n));
                        }else{
                                returnPos[0]=theta1;
                                returnPos[1]=theta2n;
                                returnPos[2]=theta3n;
                        }

                } else {
                        Serial.print("Angle out of reach");
                }
        }
}

float DynamixelClass::positionOfTime(float startAngle, float endAngle, double t, double endTime){
        return degToPos(startAngle + ((3/(endTime*endTime))*(endAngle - startAngle))*(t*t)-((2/(endTime*endTime*endTime))*(endAngle-startAngle))*(t*t*t));
}

float DynamixelClass::velocityOfTime(float startAngle, float endAngle, double t, double endTime){
        //Serial.println(abs((((6*(endAngle-startAngle)*t)/(endTime*endTime))-((6*(endAngle-startAngle)*(t*t))/(endTime*endTime*endTime)))*9.5));
        return fabs((((6*(endAngle-startAngle)*t)/(endTime*endTime))-((6*(endAngle-startAngle)*(t*t))/(endTime*endTime*endTime)))*9.5);
}
float DynamixelClass::accelerationOfTime(float startAngle, float endAngle, double t, double endTime){
        Serial.println(fabs((((6*(endAngle-startAngle))/(endTime*endTime))-((12*(endAngle-startAngle)*(t))/(endTime*endTime*endTime)))*10));
        float acceleration = fabs((((6*(endAngle-startAngle))/(endTime*endTime))-((12*(endAngle-startAngle)*(t))/(endTime*endTime*endTime)))*10);
        if (acceleration < 1) {
                acceleration = 1;
        } else {
                return acceleration;
        }
}

void DynamixelClass::jointPlanner(float x, float y, float z, float duration){

        //Read starting angles
        getPositionN();

        //Inverse calculating end angles!
        //Convert x,y,z to angles
        setEEPosition(x,y,z, false);

        Serial.print("StartAngle: ");
        Serial.println(posToDeg(returndata[0]));
        Serial.print("EndAngle: ");
        Serial.println(returnPos[0]);

        //Actually moving the robot
        for(float i = 0; i < duration; i+=0.006) {
                setGoalPosition(0x01,positionOfTime(posToDeg(returndata[0]),returnPos[0],i,duration));
                setGoalPosition(0x02,positionOfTime(posToDeg(returndata[1]),returnPos[1],i,duration));
                setGoalPosition(0x03,positionOfTime(posToDeg(returndata[2]),returnPos[2],i,duration));
        }
}

void DynamixelClass::MoveL(float x, float y, float z, float duration){
        //Read current x,y,z
        getEE();

        float vector[3] = {(x - returnPos[0]), (y - returnPos[1]), (z - returnPos[2])};

        //starting point
        float pS[3] = {returnPos[0], returnPos[1], returnPos[2]};

        for(float i = 0; i < 1; i += 0.06 / duration) {
                setEEPosition((pS[0] + vector[0] * i), (pS[1] + vector[1] * i), (pS[2] + vector[2] * i), 1);
        }
}


//      ██████╗ ███████╗████████╗████████╗███████╗██████╗ ███████╗
//    ██╔════╝ ██╔════╝╚══██╔══╝╚══██╔══╝██╔════╝██╔══██╗██╔════╝
//   ██║  ███╗█████╗     ██║      ██║   █████╗  ██████╔╝███████╗
//  ██║   ██║██╔══╝     ██║      ██║   ██╔══╝  ██╔══██╗╚════██║
// ╚██████╔╝███████╗   ██║      ██║   ███████╗██║  ██║███████║
// ╚═════╝ ╚══════╝   ╚═╝      ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝


float DynamixelClass::getVelocity(unsigned char ID){

        clearRXbuffer();
        readN(ID, 0x80, 4);             //Read from adress 0x84 (Present Position), byte size 4
        getParameters();                //Filters parameters from ReturnPacket

        float sum;
        sum = (data[2] << 8) | data[1]; //Converting two information bytes into a integer (position data)

        if(sum > 10000) {
                sum -= 65535;
        }
        sum *= 0.229 * 0.104719755;

        //Debug information
        Serial.print("Velocity:");
        Serial.print("\t");
        // Serial.print(data[0]);
        // Serial.print(" is ");
        Serial.print(sum,5);

        return sum;
}


float DynamixelClass::getCurrent(unsigned char ID){
        clearRXbuffer();
        readN(ID, 0x7E, 4);       //Read from adress 0x84 (Present Current), byte size 4
        getParameters();          //Filters parameters from ReturnPacket

        int sum;
        sum = (data[2] << 8) | data[1]; //Converting two information bytes into a integer (current data)

        //Debug information
        Serial.print("Current:");
        Serial.print("\t");
        // Serial.print(data[0]);
        // Serial.print(" is ");
        if(((sum*3.36)/1000) > 0.588){
        Serial.print(AmToNm64((sum*3.36)/1000),3);
      }else{
        Serial.print((sum*3.36)/1000);
      }

        return sum;
}

float DynamixelClass::getGoalCurrent(unsigned char ID){
        clearRXbuffer();
        readN(ID, 0x66, 4);
        getParameters();

        int sum;
        sum = (data[2] << 8 | data[1]);

        //Debug information
        Serial.print("Goal Current of ID: ");
        Serial.print(data[0]);
        Serial.print(" is ");
        Serial.println(sum);

        return sum;
}


void DynamixelClass::getParameters(void){

        int j = 0;
        for(int i = 0; i < 100; i++) {
                //Filtering the parameters from the returnpacket, by searching for the instruction (0x00, 0x55, 0x00)
                if(ReturnPacket[i] == 0x55 && ReturnPacket[i-1] == 0 && ReturnPacket[i+1] == 0) {
                        //Saving ID
                        data[j] = ReturnPacket[i-3];

                        //Saving parameter bytes
                        data[j+1] = ReturnPacket[i+2];
                        data[j+2] = ReturnPacket[i+3];

                        j += 3;
                }else if(ReturnPacket[i] == 0x55 && ReturnPacket[i-1] == 0 && ReturnPacket[i+1] != 0) {
                        debugNumber = ReturnPacket[i+1];
                }
        }

}

int DynamixelClass::getPosition(unsigned char ID){

        clearRXbuffer();
        readN(ID, 0x84, 4);             //Read from adress 0x84 (Present Position), byte size 4
        getParameters();                //Filters parameters from ReturnPacket

        int sum;
        sum = (data[2] << 8) | data[1]; //Converting two information bytes into a integer (position data)

        //Debug information
        //Serial.print("Position: ");
        // Serial.print(data[0]);
        // Serial.print(" is ");
        //Serial.println(sum);

        return sum;
}


float DynamixelClass::getPositionD(unsigned char ID){

        //Converting from raw data to degrees (360/4095)
        float posd;
        posd = getPosition(ID)*0.088*0.0174532925;
        posd -= 1.5707963;
        //Debug information
        Serial.print("Position:");
        Serial.print("\t");

        // Serial.print(ID);
        // Serial.print(" in DEG: ");
        Serial.println(posd,2);

        return posd;
}

int *DynamixelClass::getPositionN(void){

        clearRXbuffer();
        syncRN(0x84, 4);                  //Read from adress 0x84 (Present Position), byte size 4
        getParameters();                  //Filters parameters from ReturnPacket

        //Making the terminal look nice
        Serial.print("\n");
        Serial.println("__________________________________________");

        int sum;
        for(int i = 0; i < 15; i += 3) {
                sum = (data[i+2] << 8) | data[i+1]; //Converting two information bytes into a integer (position data)

                returndata[data[i]-1] = sum; //Array for storing multiple positions

                //Debug information
                Serial.print("Position of ID: ");
                Serial.print(data[i]);
                Serial.print(" is ");
                Serial.println(sum);
        }

        return returndata;                //Return pointer to array of positions
}

double *DynamixelClass::getEE(void){

        double theta1 = degToRad(posToDeg(getPosition(0x01)));
        double theta2 = degToRad(posToDeg(getPosition(0x02)));
        double theta3 = degToRad(posToDeg(getPosition(0x03)));

        double x,y,z;
        x = -0.27 * cos(theta1) * sin(theta2 + theta3) + 0.22 * cos(theta1) * sin(theta2);
        y = -0.27 * sin(theta1) * sin(theta2 + theta3) + 0.22 * sin(theta1) * sin(theta2);
        z = (0.27 * cos(theta2 + theta3)) + 0.06 - (0.22 * cos(theta2));

        returnPos[0] = x;
        returnPos[1] = y;
        returnPos[2] = z;

        //Debug information
        Serial.print("x = ");
        Serial.println(x,3);
        Serial.print("y = ");
        Serial.println(y,3);
        Serial.print("z = ");
        Serial.println(z,3);

        return returnPos;
}

float DynamixelClass::getLoad(unsigned char ID){

        clearRXbuffer();
        readN(ID, 0x7E, 4);             //Read from adress 0x7E (Present Load), byte size 4 (should be 2?)
        getParameters();                //Filters parameters from ReturnPacket

        float sum;
        sum = (data[2] << 8 | data[1]); //Converting two information bytes into a float (load data)

        //Converting load to percentage
        sum = sum / 10;

        if(sum > 100) {
                sum -= 6553.5;
        }

        //Debug information
        Serial.print("Load of ID: ");
        Serial.print(data[0]);
        Serial.print(" is ");
        Serial.println(sum);

        return sum;
}

bool DynamixelClass::getShutdown(unsigned char ID){

        clearRXbuffer();
        readN(ID, 0x3F, 4);       //Read from adress 0x3F (Shutdown byte), byte size 1
        getParameters();          //Filters parameters from ReturnPacket

        //Depending on the dedugNumber from the package, returning true or false
        //debugNumber will be 11 or 80, if the servo is overworked
        if(debugNumber > 10) {
                debugNumber = 0;
                return true;
        }
        else{
                return false;
        }
}

void DynamixelClass::rebootSequence(void){
        //We might need to implement, the other servos here

        //If the servos are overworked, they will reboot and reset the holding torque
        if(getShutdown(0x04)) {
                reboot(0x04);
                delay(100);
                setHoldingTorque(0x04, true);
                delay(100);
        }
        if(getShutdown(0x05)) {
                reboot(0x05);
                delay(100);
                setHoldingTorque(0x05, true);
                delay(100);
        }
}

double DynamixelClass::posToDeg(int pos){
        return pos * 0.0879120879;
}

double DynamixelClass::degToRad(double deg){
        return (deg * M_PI)/180;
}

double DynamixelClass::degToPos(double deg){
        return deg / 0.0879120879;
}

double DynamixelClass::NmTomAmx106(double Nm){
        return ((0.588*exp(0.3583 * Nm))*1000)/3.36;
}

double DynamixelClass::NmTomAmx64(double Nm){
        return ((0.8568 * Nm + 0.112)*1000)/3.36;
}

double DynamixelClass::AmToNm64(double Am){
        return (2.790957298*log(1.700680272*Am));
}

//      ██████╗ ███████╗ █████╗ ██████╗
//     ██╔══██╗██╔════╝██╔══██╗██╔══██╗
//    ██████╔╝█████╗  ███████║██║  ██║
//   ██╔══██╗██╔══╝  ██╔══██║██║  ██║
//  ██║  ██║███████╗██║  ██║██████╔╝
//  ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝╚═════╝


unsigned int DynamixelClass::syncWN(unsigned short addr, unsigned char*arr, int n, int dataN){

        n += 7;

        Instruction_Packet_Array[0] = 0xFE;                     //ID broadcast (253)
        Instruction_Packet_Array[1] = (n & 0xFF);               //length
        Instruction_Packet_Array[2] = (n & 0xFF00) >> 8;        //length
        Instruction_Packet_Array[3] = 0x83;                     //instruction
        Instruction_Packet_Array[4] = (addr & 0xFF);            //address
        Instruction_Packet_Array[5] = (addr & 0xFF00) >> 8;     //address
        Instruction_Packet_Array[6] = (dataN & 0xFF);           //length
        Instruction_Packet_Array[7] = (dataN & 0xFF00) >> 8;    //length

        for (int i = 0; i < n -7; i++) {
                Instruction_Packet_Array[i+8] = arr[i];
        }

        clearRXbuffer();

        transmitInstructionPacket(n);

        return 0;
}

void DynamixelClass::syncRN(unsigned short addr, int n){

        n += 8;
        Instruction_Packet_Array[0] = 0xFE;                     // ID broadcast (253)
        Instruction_Packet_Array[1] = (n & 0xFF);               //length of packet
        Instruction_Packet_Array[2] = (n & 0xFF00) >> 8;        //length of packet
        Instruction_Packet_Array[3] = 0x82;                     //Instruction
        Instruction_Packet_Array[4] = (addr & 0xFF);            //address
        Instruction_Packet_Array[5] = (addr & 0xFF00) >> 8;     //address
        Instruction_Packet_Array[6] = ((n-8) & 0xFF);           //data length
        Instruction_Packet_Array[7] = ((n-8) & 0xFF00) >> 8;    //data length

        //Ask information from ID 1 to ID 5
        Instruction_Packet_Array[8] =  0x01;                    // ID 1
        Instruction_Packet_Array[9] =  0x02;                    // ID 2
        Instruction_Packet_Array[10] = 0x03;                    // ID 3
        Instruction_Packet_Array[11] = 0x04;                    // ID 4
        Instruction_Packet_Array[12] = 0x05;                    // ID 5

        clearRXbuffer();

        transmitInstructionPacketR(n);
        readReturnPacket(); //Read return package
}


unsigned int DynamixelClass::writeN(unsigned char ID, unsigned short addr, unsigned char *arr, int n){

        n += 5;
        Instruction_Packet_Array[0] = ID;
        Instruction_Packet_Array[1] = (n & 0xFF);               //length
        Instruction_Packet_Array[2] = (n & 0xFF00) >> 8;        //length
        Instruction_Packet_Array[3] = 0x03;                     //Instruction
        Instruction_Packet_Array[4] = (addr & 0xFF);            //address
        Instruction_Packet_Array[5] = (addr & 0xFF00) >> 8;     //address

        for (int i = 0; i < n - 5; i++) {
                Instruction_Packet_Array[i+6] = arr[i];
        }

        clearRXbuffer();

        transmitInstructionPacket(n);

        return 0;
}

void DynamixelClass::readN(unsigned char ID, unsigned short addr, int n){

        n += 3;
        Instruction_Packet_Array[0] = ID;
        Instruction_Packet_Array[1] = (n & 0xFF);               //length of packet
        Instruction_Packet_Array[2] = (n & 0xFF00) >> 8;        //length of packet
        Instruction_Packet_Array[3] = 0x02;                     //Instruction
        Instruction_Packet_Array[4] = (addr & 0xFF);            //address
        Instruction_Packet_Array[5] = (addr & 0xFF00) >> 8;     //address
        Instruction_Packet_Array[6] = ((n-3) & 0xFF);           //data length
        Instruction_Packet_Array[7] = ((n-3) & 0xFF00) >> 8;    // data length

        clearRXbuffer();
        transmitInstructionPacket(n);
        readReturnPacket();
        //readAll();
}

//##############################################################################
//########################## Private Methods ###################################
//##############################################################################

void DynamixelClass::transmitInstructionPacket(int transLen){                                   // Transmit instruction packet to Dynamixel

        if (Direction_Pin > -1) {
                digitalWrite(Direction_Pin,HIGH);                                       // Set TX Buffer pin to HIGH
        }

        unsigned char arrLen = transLen+7;
        unsigned char pt[arrLen];

        pt[0] = 0xFF;
        pt[1] = 0xFF;
        pt[2] = 0xFD;
        pt[3] = 0x00;
        int i;
        for (i = 0; i <= transLen; i++) {
                pt[i+4] = Instruction_Packet_Array[i];
        }

        unsigned short crc = update_crc(pt, arrLen-2);

        unsigned char CRC_L = (crc & 0x00FF);
        unsigned char CRC_H = (crc>>8) & 0x00FF;

        i += 4;

        pt[i++] = CRC_L;
        pt[i] = CRC_H;

        for(i = 0; i < arrLen; i++) {
                _serial->write(pt[i]);
        }

        noInterrupts();

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__) // Leonardo and Mega use Serial1
        if ((UCSR1A & B01100000) != B01100000) {                                        // Wait for TX data to be sent
                _serial->flush();
        }

#elif defined(__SAM3X8E__)

        //if(USART_GetFlagStatus(USART1, USART_FLAG_TC) != RESET)
        _serial->flush();
        //}

#else
        if ((UCSR0A & B01100000) != B01100000) {                                        // Wait for TX data to be sent
                _serial->flush();
        }

#endif

        if (Direction_Pin > -1) {
                digitalWrite(Direction_Pin,LOW);                                        //Set TX Buffer pin to LOW after data has been sent
        }

        interrupts();

        delay(1);

}

void DynamixelClass::transmitInstructionPacketR(int transLen){                                   // Transmit instruction packet to Dynamixel

        if (Direction_Pin > -1) {
                digitalWrite(Direction_Pin,HIGH);                                       // Set TX Buffer pin to HIGH
        }

        unsigned char arrLen = transLen+7;
        unsigned char pt[arrLen];

        pt[0] = 0xFF;
        pt[1] = 0xFF;
        pt[2] = 0xFD;
        pt[3] = 0x00;
        int i;
        for (i = 0; i <= transLen; i++) {
                pt[i+4] = Instruction_Packet_Array[i];
        }

        unsigned short crc = update_crc(pt, arrLen-2);

        unsigned char CRC_L = (crc & 0x00FF);
        unsigned char CRC_H = (crc>>8) & 0x00FF;

        i += 4;

        pt[i++] = CRC_L;
        pt[i] = CRC_H;

        for(i = 0; i < arrLen; i++) {
                _serial->write(pt[i]);
        }

        noInterrupts();

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__) // Leonardo and Mega use Serial1
        if ((UCSR1A & B01100000) != B01100000) {                                        // Wait for TX data to be sent
                _serial->flush();
        }

#elif defined(__SAM3X8E__)

        //if(USART_GetFlagStatus(USART1, USART_FLAG_TC) != RESET)
        _serial->flush();
        //}

#else
        if ((UCSR0A & B01100000) != B01100000) {                                        // Wait for TX data to be sent
                _serial->flush();
        }

#endif

        if (Direction_Pin > -1) {
                digitalWrite(Direction_Pin,LOW);                                        //Set TX Buffer pin to LOW after data has been sent
        }

        interrupts();

        delay(5);

}

void DynamixelClass::readReturnPacket(void){

        int i = 0;
        //Read information when available
        while(_serial->available()>0) {
                int incomingbyte;
                incomingbyte = _serial->read(); //Save incomingbyte

                ReturnPacket[i]=incomingbyte; //Save data in ReturnPacket array
                i++;
        }
}

void DynamixelClass::readAll(void){
        //Print all incomming serial data, when available

        while(_serial->available()>0) {
                int incomingbyte;
                incomingbyte = _serial->read();

                Serial.println(incomingbyte, HEX); //Print incomingbyte, one by one - in HEX
        }
}

void DynamixelClass::gripper(void){
        for(int i = 0; i<400; i++) {
                setGoalPosition(0x04, 2298-i);
                setGoalPosition(0x05, 1798+i);
                if(getLoad(0x04) < -40) {
                        break;
                }
                if(getLoad(0x05) > 40) {
                        break;
                }
                Serial.println(i);
        }
}


void DynamixelClass::moveAll(void){

        readXbee();
        //Switch between linear and joint space moving
        if (EMG[0] >= EMG1Hi && peakMode == 0) {
                long peakTime1 = millis();
                if (peakTime1 - peakTime2 < 1000) {
                        if (moveLin == false) {
                                moveLin = true;
                        } else {
                                moveLin = false;
                        }
                }
                peakMode = 1;
                peakTime2 = peakTime1;
        } else if (EMG[0] < EMG1Low * 5) {
                peakMode = 0;
        }

        //Changing state of the gripper
        if (EMG[1] >= EMG2Hi && peak3 == 0) {
                long peakTime5 = millis();
                if (peakTime5 - peakTime6 < 1000) {
                        if (gripperState == LOW) {
                                setGoalPosition(0x04, 2048+250);
                                setGoalPosition(0x05, 2048-250);
                                gripperState = HIGH;
                        } else {
                                gripper();
                                gripperState = LOW;
                        }
                }
                peak3 = 1;
                peakTime6 = peakTime5;
        } else if (EMG[1] < EMG2Low * 5) {
                peak3 = 0;
        }

        if(moveLin) {
                if(linearInit) {
                        getEE();
                        endPosXYZ[0] = returnPos[0];
                        endPosXYZ[1] = returnPos[1];
                        endPosXYZ[2] = returnPos[2];
                        linearInit = false;
                }


                // Serial.print(smoothY[0]);
                // Serial.print("\t");
                // Serial.println(smoothZ[0]);

                //Incrementing the cartesian directions
                endPosXYZ[0] = linearMoving('x', smoothEMG1[0], EMG1Low, EMG1Hi, 0.001, endPosXYZ[0]);
                endPosXYZ[1] = linearMoving('y', smoothEMG2[0], EMG2Low, EMG2Hi, 0.001, endPosXYZ[1]);
                endPosXYZ[2] = linearMovingZ(smoothY[0], smoothZ[0], 0.001, endPosXYZ[2]);

                //Debud information
                Serial.print("endPos X: ");
                Serial.println(endPosXYZ[0]);
                Serial.print("endPos y: ");
                Serial.println(endPosXYZ[1]);
                Serial.print("endPos z: ");
                Serial.println(endPosXYZ[2]);

                //Only move if one of the direction needs to change positively or negatively
                if((moveX & 0b1000) >> 3 || (moveX & 0b0100) >> 2 ||
                   (moveY & 0b0100) >> 2 || (moveY & 0b1000) >> 3 ||
                   (moveZ & 0b0100) >> 2 || (moveZ & 0b1000) >> 3) {
                        unsigned long moveTime = millis();
                        if(moveTime - moveTimeprevious3 > 5) {
                                moveTimeprevious3 = moveTime;
                                setEEPosition(endPosXYZ[0], endPosXYZ[1], endPosXYZ[2], true);
                        }
                }

        }else{
                linearInit = true;
                //Debug information
                Serial.print(smoothEMG1[0]);
                Serial.print("\t");
                Serial.println(smoothEMG2[0]);

                //Moving joint 1 through 3
                jointMoveingRotate(smoothY[0], smoothZ[0], 50);
                jointMoving(0x02,smoothEMG1[0],EMG1Low,EMG1Hi,100);
                jointMoving(0x03,smoothEMG2[0],EMG2Low,EMG2Hi,100);
        }
}

void DynamixelClass::jointMoving(unsigned char ID, float inputEMG, float lowThreshold, float highThreshold, float increment){
        //Setting up local variables
        bool movePLocal;
        bool moveNLocal;
        bool resetTimeLocal1;
        bool resetTimeLocal2;
        float oldTimeLocal1;
        float oldTimeLocal2;
        float oldMoveTimeLocal;

        //Loading global variables into the local ones
        if(ID == 0x02) {
                movePLocal = (joint2 & 0b1000) >> 3;
                moveNLocal = (joint2 & 0b0100) >> 2;
                resetTimeLocal1 = (joint2 & 0b0010) >> 1;
                resetTimeLocal2 = joint2 & 0b0001;
                oldTimeLocal1 = previousMillis1;
                oldTimeLocal2 = previousMillis2;
                oldMoveTimeLocal = moveTimeprevious1;
        } else {
                movePLocal = (joint3 & 0b1000) >> 3;
                moveNLocal = (joint3 & 0b0100) >> 2;
                resetTimeLocal1 = (joint3 & 0b0010) >> 1;
                resetTimeLocal2 = joint3 & 0b0001;
                oldTimeLocal1 = previousMillis3;
                oldTimeLocal2 = previousMillis4;
                oldMoveTimeLocal = moveTimeprevious2;
        }

        int endPos = getPosition(ID);

        //Incrementing positively
        if(inputEMG > lowThreshold && inputEMG < highThreshold) {
                moveNLocal = false;
                resetTimeLocal2 = true;

                if(movePLocal == false) {
                        unsigned long currentMillis1 = millis();
                        if(resetTimeLocal1) {
                                oldTimeLocal1 = currentMillis1;
                                resetTimeLocal1 = false;
                        }
                        if(currentMillis1 - oldTimeLocal1 > 500) {
                                oldTimeLocal1 = currentMillis1;
                                movePLocal = true;
                                Serial.print("move positive joint: ");
                                Serial.println(ID);
                        }
                }else{
                        endPos += increment;
                        goto end;
                }
        }
        //Incrementing negatively
        if(inputEMG > highThreshold) {
                movePLocal = false;
                resetTimeLocal1 = true;

                if(moveNLocal == false) {
                        unsigned long currentMillis2 = millis();
                        if(resetTimeLocal2) {
                                oldTimeLocal2 = currentMillis2;
                                resetTimeLocal2 = false;
                        }
                        if(currentMillis2 - oldTimeLocal2 > 500) {
                                oldTimeLocal2 = currentMillis2;
                                moveNLocal = true;
                                Serial.print("move nigative joint: ");
                                Serial.println(ID);
                        }
                }else{
                        endPos -= increment;
                        goto end;
                }
        }
        if(inputEMG < lowThreshold) {
                movePLocal = false;
                moveNLocal = false;
                resetTimeLocal1 = true;
                resetTimeLocal2 = true;
        }

end:    ;

        //Actually moving
        if(movePLocal || moveNLocal) {
                unsigned long moveTime = millis();
                if(moveTime - oldMoveTimeLocal > 10) {
                        oldMoveTimeLocal = moveTime;
                        setGoalPosition(ID, endPos);
                }
        }

        //Saving the local variables into the global ones
        if(ID == 0x02) {
                joint2 = (movePLocal<<3);
                joint2 |= (moveNLocal<<2);
                joint2 |= (resetTimeLocal1<<1);
                joint2 |= resetTimeLocal2;
                previousMillis1 = oldTimeLocal1;
                previousMillis2 = oldTimeLocal2;
                moveTimeprevious1 = oldMoveTimeLocal;
        } else {
                joint3 = (movePLocal<<3);
                joint3 |= (moveNLocal<<2);
                joint3 |= (resetTimeLocal1<<1);
                joint3 |= resetTimeLocal2;
                previousMillis3 = oldTimeLocal1;
                previousMillis4 = oldTimeLocal2;
                moveTimeprevious2 = oldMoveTimeLocal;
        }
}

void DynamixelClass::jointMoveingRotate(float inputAccY, float inputAccZ, float increment){
        //Setting up local variables
        bool movePLocal;
        bool moveNLocal;
        bool resetTimeLocal1;
        bool resetTimeLocal2;
        float oldTimeLocal1;
        float oldTimeLocal2;
        float oldMoveTimeLocal;

        //Loading global variables into the local ones
        movePLocal = (joint1 & 0b1000) >> 3;
        moveNLocal = (joint1 & 0b0100) >> 2;
        resetTimeLocal1 = (joint1 & 0b0010) >> 1;
        resetTimeLocal2 = joint1 & 0b0001;
        oldTimeLocal1 = previousMillis5;
        oldTimeLocal2 = previousMillis6;
        oldMoveTimeLocal = moveTimeprevious3;

        int endPos = getPosition(0x01);

        //Incrementing positively
        if(inputAccY < 600 && inputAccZ < 530)  {
                moveNLocal = false;
                resetTimeLocal2 = true;
                if(movePLocal == false) {
                        unsigned long currentMillis1 = millis();
                        if(resetTimeLocal1) {
                                oldTimeLocal1 = currentMillis1;
                                resetTimeLocal1 = false;
                        }
                        if(currentMillis1 - oldTimeLocal1 > 500) {
                                oldTimeLocal1 = currentMillis1;
                                movePLocal = true;
                                Serial.print("Rotate positive");
                        }
                }else{
                        endPos += increment;
                        goto end;
                }
        }
        //Incrementing negatively
        if(inputAccZ > 600) {
                movePLocal = false;
                resetTimeLocal1 = true;

                if(moveNLocal == false) {
                        unsigned long currentMillis2 = millis();
                        if(resetTimeLocal2) {
                                oldTimeLocal2 = currentMillis2;
                                resetTimeLocal2 = false;
                        }
                        if(currentMillis2 - oldTimeLocal2 > 500) {
                                oldTimeLocal2 = currentMillis2;
                                moveNLocal = true;
                                Serial.print("move negative: ");
                                Serial.println("Z");
                        }
                }else{
                        endPos -= increment;
                        goto end;
                }
        }
        if(inputAccY > 600 && inputAccZ < 600) {
                movePLocal = false;
                moveNLocal = false;
                resetTimeLocal1 = true;
                resetTimeLocal2 = true;
        }

end:    ;

//Actually moving
if(movePLocal || moveNLocal) {
        unsigned long moveTime = millis();
        if(moveTime - oldMoveTimeLocal > 10) {
                oldMoveTimeLocal = moveTime;
                setGoalPosition(0x01, endPos);
        }
}

        //Saving the local variables into the global ones
        joint1 = (movePLocal<<3);
        joint1 |= (moveNLocal<<2);
        joint1 |= (resetTimeLocal1<<1);
        joint1 |= resetTimeLocal2;
        previousMillis5 = oldTimeLocal1;
        previousMillis6 = oldTimeLocal2;
        moveTimeprevious3 = oldMoveTimeLocal;
}

double DynamixelClass::linearMoving(char direction, float inputEMG, float lowThreshold, float highThreshold, float increment, double startingPosition){
        //Setting up local variables
        bool movePLocal;
        bool moveNLocal;
        bool resetTimeLocal1;
        bool resetTimeLocal2;
        float oldTimeLocal1;
        float oldTimeLocal2;
        int getter;

        //Loading global variables into the local ones
        if(direction == 'x' || direction == 'X') {
                movePLocal = (moveX & 0b1000) >> 3;
                moveNLocal = (moveX & 0b0100) >> 2;
                resetTimeLocal1 = (moveX & 0b0010) >> 1;
                resetTimeLocal2 = moveX & 0b0001;
                oldTimeLocal1 = previousMillis1;
                oldTimeLocal2 = previousMillis2;
                getter = 0;
        } else if(direction == 'y' || direction == 'Y') {
                movePLocal = (moveY & 0b1000) >> 3;
                moveNLocal = (moveY & 0b0100) >> 2;
                resetTimeLocal1 = (moveY & 0b0010) >> 1;
                resetTimeLocal2 = moveY & 0b0001;
                oldTimeLocal1 = previousMillis3;
                oldTimeLocal2 = previousMillis4;
                getter = 1;
        }else{
                Serial.println("Error: input x or y");
        }

        //Incrementing positively
        if(inputEMG > lowThreshold && inputEMG < highThreshold) {
                moveNLocal = false;
                resetTimeLocal2 = true;

                if(movePLocal == false) {
                        unsigned long currentMillis1 = millis();
                        if(resetTimeLocal1) {
                                oldTimeLocal1 = currentMillis1;
                                resetTimeLocal1 = false;
                        }
                        if(currentMillis1 - oldTimeLocal1 > 500) {
                                oldTimeLocal1 = currentMillis1;
                                movePLocal = true;
                                Serial.print("move positive: ");
                                Serial.println(direction);
                        }
                }else{
                        endPosXYZ[getter] = startingPosition + increment;
                        if(endPosXYZ[getter] > 0.49) {
                                endPosXYZ[getter] = 0.49;
                        }
                        goto end;
                }
        }
        //Incrementing negatively
        if(inputEMG > highThreshold) {
                movePLocal = false;
                resetTimeLocal1 = true;

                if(moveNLocal == false) {
                        unsigned long currentMillis2 = millis();
                        if(resetTimeLocal2) {
                                oldTimeLocal2 = currentMillis2;
                                resetTimeLocal2 = false;
                        }
                        if(currentMillis2 - oldTimeLocal2 > 500) {
                                oldTimeLocal2 = currentMillis2;
                                moveNLocal = true;
                                Serial.print("move negative: ");
                                Serial.println(direction);
                        }
                }else{
                        endPosXYZ[getter] = startingPosition - increment;
                        if(endPosXYZ[getter] < -0.49) {
                                endPosXYZ[getter] = -0.49;
                        }
                        goto end;
                }
        }
        if(inputEMG < lowThreshold) {
                movePLocal = false;
                moveNLocal = false;
                resetTimeLocal1 = true;
                resetTimeLocal2 = true;
        }

end:    ;

        //Saving the local variables into the global ones
        if(direction == 'x' || direction == 'X') {
                moveX = (movePLocal<<3);
                moveX |= (moveNLocal<<2);
                moveX |= (resetTimeLocal1<<1);
                moveX |= resetTimeLocal2;
                previousMillis1 = oldTimeLocal1;
                previousMillis2 = oldTimeLocal2;
        } else if(direction == 'y' || direction == 'Y') {
                moveY = (movePLocal<<3);
                moveY |= (moveNLocal<<2);
                moveY |= (resetTimeLocal1<<1);
                moveY |= resetTimeLocal2;
                previousMillis3 = oldTimeLocal1;
                previousMillis4 = oldTimeLocal2;
        }else{
                Serial.println("Error: input x or y");
        }
        return endPosXYZ[getter];
}

double DynamixelClass::linearMovingZ(float inputAccY, float inputAccZ, float increment, double startingPosition){
        //Setting up local variables
        bool movePLocal;
        bool moveNLocal;
        bool resetTimeLocal1;
        bool resetTimeLocal2;
        float oldTimeLocal1;
        float oldTimeLocal2;
        int getter;

        //Loading global variables into the local ones
        movePLocal = (moveZ & 0b1000) >> 3;
        moveNLocal = (moveZ & 0b0100) >> 2;
        resetTimeLocal1 = (moveZ & 0b0010) >> 1;
        resetTimeLocal2 = moveZ & 0b0001;
        oldTimeLocal1 = previousMillis5;
        oldTimeLocal2 = previousMillis6;
        getter = 2;

        //Incrementing positively
        if(inputAccY < 600 && inputAccZ < 530)  {
                moveNLocal = false;
                resetTimeLocal2 = true;
                if(movePLocal == false) {
                        unsigned long currentMillis1 = millis();
                        if(resetTimeLocal1) {
                                oldTimeLocal1 = currentMillis1;
                                resetTimeLocal1 = false;
                        }
                        if(currentMillis1 - oldTimeLocal1 > 500) {
                                oldTimeLocal1 = currentMillis1;
                                movePLocal = true;
                                Serial.print("move positive: ");
                                Serial.println("Z");
                        }
                }else{
                        endPosXYZ[getter] = startingPosition + increment;
                        if(endPosXYZ[getter] > 0.55) {
                                endPosXYZ[getter] = 0.55;
                        }
                        goto end;
                }
        }
        //Incrementing negatively
        if(inputAccZ > 600) {
                movePLocal = false;
                resetTimeLocal1 = true;

                if(moveNLocal == false) {
                        unsigned long currentMillis2 = millis();
                        if(resetTimeLocal2) {
                                oldTimeLocal2 = currentMillis2;
                                resetTimeLocal2 = false;
                        }
                        if(currentMillis2 - oldTimeLocal2 > 500) {
                                oldTimeLocal2 = currentMillis2;
                                moveNLocal = true;
                                Serial.print("move negative: ");
                                Serial.println("Z");
                        }
                }else{
                        endPosXYZ[getter]= startingPosition - increment;
                        if(endPosXYZ[getter] < -0.15) {
                                endPosXYZ[getter] = -0.15;
                        }
                        goto end;
                }
        }
        if(inputAccY > 600 && inputAccZ < 600) {
                movePLocal = false;
                moveNLocal = false;
                resetTimeLocal1 = true;
                resetTimeLocal2 = true;
        }

end:    ;

        //Saving the local variables into the global ones
        moveZ = (movePLocal<<3);
        moveZ |= (moveNLocal<<2);
        moveZ |= (resetTimeLocal1<<1);
        moveZ |= resetTimeLocal2;
        previousMillis5 = oldTimeLocal1;
        previousMillis6 = oldTimeLocal2;

        return endPosXYZ[getter];
}

void DynamixelClass::writeXbee(void){

        while(_serial2->available()>=24) {
                unsigned char incomingbyte1;
                int sum = 0;
                incomingbyte1 = _serial2->read();
                if (incomingbyte1 == 126) {
                        EMGReturn[0] = incomingbyte1;
                        for(int i = 1; i<=23; i++) {
                                incomingbyte1 = _serial2->read();
                                if(incomingbyte1 == 126) {
                                        EMGReturn[0] = incomingbyte1;
                                        i = 1;
                                }else{
                                        EMGReturn[i] = incomingbyte1;
                                }
                        }

                        for(int i = 3; i<23; i++) {
                                sum += EMGReturn[i];
                        }

                        sum %= 256;
                        if (EMGReturn[23] == 255-sum) {
                                for(int i = 0; i<=23; i++) {
                                        Serial.write(EMGReturn[i]);
                                }
                        }
                }
        }
}

float DynamixelClass::readXbee(void){
        float beta1 = 0.25;
        float beta2 = 0.45;

        while(_serial2->available()>=24) {
                unsigned char incomingbyte1;
                int sum = 0;
                incomingbyte1 = _serial2->read();
                if (incomingbyte1 == 126) {
                        EMGReturn[0] = incomingbyte1;
                        for(int i = 1; i<=23; i++) {
                                incomingbyte1 = _serial2->read();
                                if(incomingbyte1 == 126) {
                                        EMGReturn[0] = incomingbyte1;
                                        i = 1;
                                }else{
                                        EMGReturn[i] = incomingbyte1;
                                }
                        }

                        for(int i = 3; i<23; i++) {
                                sum += EMGReturn[i];
                        }

                        sum %= 256;
                        if (EMGReturn[23] == 255-sum) {
                                int sumZ;
                                int sumY;
                                int sumX;
                                sumZ = (EMGReturn[13] << 8) | EMGReturn[14];
                                sumY = (EMGReturn[15] << 8) | EMGReturn[16];
                                sumX = (EMGReturn[17] << 8) | EMGReturn[18];

                                acc[0]=sumX;
                                acc[1]=sumY;
                                acc[2]=sumZ;
                                //Sum of the low and high byte for the 2 EMG channels.
                                EMG[0] = (EMGReturn[19] << 8) | EMGReturn[20];
                                EMG[1] = (EMGReturn[21] << 8) | EMGReturn[22];
                        }
                }
        }
        delay(7);

        //Applying digital low pass filter, on EMG and accelerometer data
        smoothEMG1[1] = smoothEMG1[0] - (beta1 * (smoothEMG1[1] - EMG[0]));
        smoothEMG1[0] = smoothEMG1[1];

        smoothEMG2[1] = smoothEMG2[0] - (beta1 * (smoothEMG2[1] - EMG[1]));
        smoothEMG2[0] = smoothEMG2[1];

        smoothX[1] = smoothX[0] - (beta2 * (smoothX[1] - acc[0]));
        smoothX[0] = smoothX[1];

        smoothY[1] = smoothY[0] - (beta2 * (smoothY[1] - acc[1]));
        smoothY[0] = smoothY[1];

        smoothZ[1] = smoothZ[0] - (beta2 * (smoothZ[1] - acc[2]));
        smoothZ[0] = smoothZ[1];

        // Serial.print(smoothEMG1[0]);
        // Serial.print("\t");
        // Serial.println(smoothEMG2[0]);


        return EMG[1];
}

void DynamixelClass::reboot(unsigned char ID){
        Instruction_Packet_Array[0] = ID;
        Instruction_Packet_Array[1] = (3 & 0xFF);             //length
        Instruction_Packet_Array[2] = (3 & 0xFF00) >> 8;      //length
        Instruction_Packet_Array[3] = 0x08;                   //Instruction

        clearRXbuffer();
        transmitInstructionPacket(3);
        readAll();
}

float DynamixelClass::slope(void){
        v.push_back(nubmerOfPackets);
        v.push_back(smoothX[0]);
        v.push_back(smoothY[0]);
        v.push_back(smoothZ[0]);
        v.push_back(smoothEMG1[0]);
        v.push_back(smoothEMG2[0]);
        if(v.size()>30) {
                v.erase(v.begin(),v.begin()+6);
        }
        nubmerOfPackets++;
        if(v.size()==30) {
                float m[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
                float mx = 0;
                float my = 0;
                float mz = 0;
                float memg1 = 0;
                float memg2 = 0;
                for(int i = 0; i<6; i++) {
                        m[i]=v.at(i);
                        m[i+6]=v.at(i+24);
                }
                mx = (m[1]-m[7])/(m[0]-m[6]);
                my = (m[2]-m[8])/(m[0]-m[6]);
                mz = (m[3]-m[9])/(m[0]-m[6]);
                memg1 = (m[4]-m[10])/(m[0]-m[6]);
                memg2 = (m[5]-m[11])/(m[0]-m[6]);
                slop[0] = mx;
                slop[1] = my;
                slop[2] = mz;
                slop[3] = memg1;
                slop[4] = memg2;
                return slop[3];
        }
}

void DynamixelClass::Calibration(void){

        int max1 = 0;
        int max2 = 0;

        Serial.println("Start calibration");
        Serial.println("Waiting for EMG1");

        int i = 0;
        while(i < 4) {
                readXbee();

                switch(i) {
                case 0:
                        if(smoothEMG1[0] > 40) {
                                Serial.println("EMG1 Detected");
                                i++;
                        }
                        break;
                case 1:
                        Serial.println(smoothEMG1[0]);
                        //Save the highest value
                        if(smoothEMG1[0] > max1) {
                                max1 = smoothEMG1[0];
                        }
                        //When relaxed
                        if(smoothEMG1[0] < 40 ) {
                                Serial.println("EMG1 Calibrated");
                                Serial.println("Waiting for EMG2");
                                i++;
                        }
                        break;
                case 2:
                        if(smoothEMG2[0] > 40) {
                                Serial.println("EMG2 Detected");
                                i++;
                        }
                        break;

                case 3:
                        Serial.println(smoothEMG2[0]);
                        //Save the highest value
                        if(smoothEMG2[0] > max2) {
                                max2 = smoothEMG2[0];
                        }
                        //When relaxed
                        if(smoothEMG2[0] < 40) {
                                Serial.println("EMG2 Calibrated");
                                i++;
                        }
                        break;
                }
        }

        Serial.print(max1);
        Serial.print("\t");
        Serial.println(max2);

        EMG1Max = max1 * 0.7;
        EMG1Hi = max1 * 0.2;
        EMG1Low  = EMG1Hi * 0.1;
        EMG2Max = max2 * 0.7;
        EMG2Hi = max2 * 0.2;
        EMG2Low  = EMG2Hi * 0.1;

        Serial.print("The flip threshold of the first muscle is: ");
        Serial.println(EMG1Hi);

        Serial.print("The activation threshold of the first muscle is: ");
        Serial.println(EMG1Low);

        Serial.print("The flip threshold of the second muscle is: ");
        Serial.println(EMG2Hi);

        Serial.print("The activation threshold of the second muscle is: ");
        Serial.println(EMG2Low);
}


void DynamixelClass::clearRXbuffer(void){

        while (_serial->read() != -1);  // Clear RX buffer;
}


unsigned short DynamixelClass::update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
        unsigned short crc_accum = 0;
        unsigned short i, j;
        unsigned short crc_table[256] = {
                0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
                0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
                0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
                0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
                0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
                0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
                0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
                0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
                0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
                0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
                0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
                0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
                0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
                0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
                0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
                0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
                0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
                0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
                0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
                0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
                0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
                0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
                0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
                0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
                0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
                0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
                0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
        };

        for(j = 0; j < data_blk_size; j++)
        {
                i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
                crc_accum = (crc_accum << 8) ^ crc_table[i];
        }

        return crc_accum;
}

DynamixelClass Dynamixel;
