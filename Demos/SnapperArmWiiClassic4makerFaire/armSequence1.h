//Arm 5
//Sequence 4
//Mode 2
//Orientation 1
//DIO 1
#include "Kinematics.h"
#include "GlobalArm.h"
extern void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable);
// We need to declare the data exchange
// variable to be volatile - the value is
// read from memory.
volatile int playState1 = 0; // 0 = stopped 1 = playing

void playSequence1()
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
  playState1 = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_CYLINDRICAL;
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    //DIO1
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    digitalWrite(4, LOW);
    digitalWrite(2, HIGH);
    IKSequencingControl(1500 , 150 , 150 , 0 , 0 , 1500 , 2000 , 1000, playState1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 2
    //###########################################################// 
    //DIO1
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    digitalWrite(4, LOW);
    digitalWrite(2, HIGH);
    IKSequencingControl(1524 , 173 , 44 , -30 , 0 , 2400 , 2000 , 1000, playState1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 3
    //###########################################################// 
    //DIO1
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    digitalWrite(4, LOW);
    digitalWrite(2, HIGH);
    IKSequencingControl(1524 , 173 , 44 , -30 , 0 , 1030 , 2000 , 1000, playState1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 4
    //###########################################################// 
    //DIO1
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    digitalWrite(4, LOW);
    digitalWrite(2, HIGH);
    IKSequencingControl(1524 , 119 , 225 , -2 , 0 , 1030 , 2000 , 1000, playState1);
    digitalWrite(2, LOW);
    //###########################################################// 

 delay(100);
 Serial.println("Pausing Sequencing Mode."); 
 delay(500);
 //uncomment this to  put the arm in sleep position after a sequence
 //PutArmToSleep();
}
