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
volatile int playState2 = 0; // 0 = stopped 1 = playing

void playSequence2()
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
  playState2 = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_CYLINDRICAL;
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    //DIO2
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(2, LOW);
    IKSequencingControl(1909 , 200 , 51 , -16 , 0 , 1375 , 2000 , 1000, playState2);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 2
    //###########################################################// 
    //DIO2
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(2, LOW);
    IKSequencingControl(1909 , 200 , 51 , -16 , 0 , 2400 , 2000 , 1000, playState2);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 3
    //###########################################################// 
    //DIO2
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(2, LOW);
    IKSequencingControl(1909 , 176 , 153 , -1 , 0 , 2400 , 2000 , 1000, playState2);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 4
    //###########################################################// 
    //DIO2
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(2, LOW);
    IKSequencingControl(1909 , 176 , 153 , -1 , 0 , 2400 , 2000 , 1000, playState2);
    digitalWrite(4, LOW);
    //###########################################################// 

 delay(100);
 Serial.println("Pausing Sequencing Mode."); 
 delay(500);
 //uncomment this to  put the arm in sleep position after a sequence
 //PutArmToSleep();
}
