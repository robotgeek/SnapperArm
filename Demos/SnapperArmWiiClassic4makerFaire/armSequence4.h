//Arm 5
//Sequence 3
//Mode 3
//Orientation 1
//DIO 1
#include "Kinematics.h"
#include "GlobalArm.h"
extern void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable);
// We need to declare the data exchange
// variable to be volatile - the value is
// read from memory.
volatile int playState4 = 0; // 0 = stopped 1 = playing

void playSequence4()
{
    pinMode(2, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
  delay(500);
  Serial.println("Sequencing Mode Active."); 
  Serial.println("Press Pushbutton  to stop");
  playState4 = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_BACKHOE;
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    //DIO8
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(4, LOW);
    digitalWrite(2, LOW);
    IKSequencingControl(1500 , 1073 , 1944 , 1500 , 1500 , 1313 , 2000 , 1000, playState4);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 2
    //###########################################################// 
    //DIO8
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(4, LOW);
    digitalWrite(2, LOW);
    IKSequencingControl(1500 , 1073 , 1944 , 1500 , 1500 , 2400 , 2000 , 1000, playState4);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 3
    //###########################################################// 
    //DIO8
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(4, LOW);
    digitalWrite(2, LOW);
    IKSequencingControl(1500 , 948 , 1339 , 1067 , 1500 , 2400 , 2000 , 1000, playState4);
    digitalWrite(8, LOW);
    //###########################################################// 

 delay(100);
 Serial.println("Pausing Sequencing Mode."); 
 delay(500);
 //uncomment this to  put the arm in sleep position after a sequence
 //PutArmToSleep();
}
