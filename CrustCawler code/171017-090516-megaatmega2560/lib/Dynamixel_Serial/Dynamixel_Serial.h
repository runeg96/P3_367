#ifndef Dynamixel_Serial_h
#define Dynamixel_Serial_h

#include "StandardCplusplus.h"
#include "vector"
#include "Wire.h"

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif


#define NONE                            0x00
#define READ                            0x01
#define ALL                             0x02

#define STATUS_PACKET_TIMEOUT           50
#define STATUS_FRAME_BUFFER             5

class DynamixelClass {

public:
// Constructor
DynamixelClass() : Direction_Pin(-1), Status_Return_Value(READ) {
}

void begin(long baud, long baud2);
void begin(HardwareSerial&, long);
void begin(Stream&);
void end(void);

unsigned int writeN(unsigned char ID, unsigned short addr, unsigned char *arr, int n);
unsigned int syncWN(unsigned short addr, unsigned char*arr, int n, int dataN);
void readN(unsigned char ID, unsigned short addr, int n);
void syncRN(unsigned short addr, int n);

//Setters
void setHoldingTorque(unsigned char ID, bool Set);
void setDirectionPin(unsigned char);
void setLimits(unsigned char ID, int max, int min);
void setOperationMode(unsigned char ID, unsigned int mode);
void setBaudrate(unsigned char ID, unsigned int baud);
void setDriveMode(unsigned char ID, bool direction);
void setGoalPosition(unsigned char ID, unsigned int pos);
void setNGoalPositions(int m1, int m2, int m3, int m4, int m5);
void setGoalVelocity(unsigned char ID, unsigned int vel);
void setProfileAcceleration(unsigned char ID, unsigned int pac);
void setProfileVelocity(unsigned char ID, unsigned int pvl);
void setGoalCurrent(unsigned char ID, unsigned int current);
void setEEPosition(double x, double y, double z, bool move);

//Getters
int  getPosition(unsigned char ID);
float getPositionD(unsigned char ID);
int *getPositionN(void);
float getLoad(unsigned char ID);
float getCurrent(unsigned char ID);
float getGoalCurrent(unsigned char ID);
double *getEE(void);
float getVelocity(unsigned char ID);


double NmTomAmx106(double Nm);
double NmTomAmx64(double Nm);
double AmToNm64(double Am);

float positionOfTime(float startAngle, float endAngle, double time, double endTime);
float velocityOfTime(float startAngle, float endAngle, double t, double endTime);
float accelerationOfTime(float startAngle, float endAngle, double t, double endTime);
void jointPlanner(float x, float y, float z, float duration);
void MoveL(float x, float y, float z, float duration);
float readXbee(void);
void moveAll(void);
void readAll(void);
void writeXbee(void);
float slope(void);
void reboot(unsigned char ID);
void Calibration(void);
void gripper(void);

Stream *_serial;
Stream *_serial2;
Stream *_serial3;

double posToDeg(int pos);
double degToRad(double deg);
double degToPos(double deg);

private:
unsigned short update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size);
void transmitInstructionPacket(int transLen);
void transmitInstructionPacketR(int transLen);
void readReturnPacket(void);
void clearRXbuffer(void);

//Converters

void getParameters(void);
bool getShutdown(unsigned char ID);
void rebootSequence(void);

//Moving in joint and cartesian space
void jointMoveingRotate(float inputAccY, float inputAccZ, float increment);
void jointMoving(unsigned char ID, float inputEMG, float lowThreshold, float highThreshold, float increment);
double linearMoving(char direction, float inputEMG, float lowThreshold, float highThreshold, float increment, double startingPosition);
double linearMovingZ(float inputAccY, float inputAccZ, float increment, double startingPosition);

//Members
unsigned char Instruction_Packet_Array[64];         // Array to hold instruction packet data
unsigned int ReturnPacket[100];                     // Array to hold returned status packet data
char Direction_Pin;                                 // Pin to control TX/RX buffer chip
unsigned char Status_Return_Value;                  // Status packet return states ( NON , READ , ALL )

unsigned int data[15];                              // Data from ReturnPacket
int returndata[5];
double returnPos[3];                                // Data return
unsigned char EMGReturn[24];
int EMG[2];
int acc[3];

float smoothEMG1[2] = {0,0};
float smoothEMG2[2] = {0,0};
float smoothX[2] = {0,0};
float smoothY[2] = {0,0};
float smoothZ[2] = {0,0};

int nubmerOfPackets = 0;
float slop[5] = {0,0,0,0,0};
std::vector<float> v;

//time delay variables
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
unsigned long previousMillis4 = 0;
unsigned long previousMillis5 = 0;
unsigned long previousMillis6 = 0;
unsigned long moveTimeprevious1 = 0;
unsigned long moveTimeprevious2 = 0;
unsigned long moveTimeprevious3 = 0;

int gripperState = LOW;
long peakTime2 = 0;
long peakTime6 = 0;
long Time1 = 0;
long Time2 = 0;
int peakMode = 0;
int peak3 = 0;

//EMG threshold
float EMG1Hi = 0;
float EMG1Low  = 0;
float EMG1Max = 1000;
float EMG2Hi = 0;
float EMG2Low  = 0;
float EMG2Max = 1000;

bool moveLin = false;

bool linearInit = true;

double endPosXYZ[3] = {0,0,0};

//Char bit arrays for moving
char joint1 = 0b0011;
char joint2 = 0b0011;
char joint3 = 0b0011;
char moveX = 0b0011;
char moveY = 0b0011;
char moveZ = 0b0011;

unsigned int debugNumber;

};


extern DynamixelClass Dynamixel;

#endif
