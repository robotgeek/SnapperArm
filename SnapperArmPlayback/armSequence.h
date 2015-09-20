//Arm 5
//Sequence 3
//Mode 1
//Orientation 1
#include "Kinematics.h"
#include "GlobalArm.h"
extern void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable);
// We need to declare the data exchange
// variable to be volatile - the value is
// read from memory.
volatile int playState = 0; // 0 = stopped 1 = playing

void playSequence()
{
  delay(500);
  Serial.println("Sequencing Mode Active."); 
  Serial.println("Press Pushbutton  to stop");
  playState = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_IK3D_CARTESIAN;
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 2
    //###########################################################// 
    IKSequencingControl(-80 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 3
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 4
    //###########################################################// 
    IKSequencingControl(0 , 200 , 150 , 0 , 0 , 256 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 5
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 6
    //###########################################################// 
    IKSequencingControl(0 , 150 , 225 , 0 , 0 , 256 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 7
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 8
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 30 , 0 , 256 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 9
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 10
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 512 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 11
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 0 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 12
    //###########################################################// 
    IKSequencingControl(100 , 200 , 200 , 0 , 0 , 512 , 4080 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 13
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 0 , 2000 , 0, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 14
    //###########################################################// 
    IKSequencingControl(100 , 200 , 200 , 0 , 0 , 10 , 160 , 0, playState);
    //###########################################################// 

 delay(100);
 Serial.println("Pausing Sequencing Mode."); 
 delay(500);
 //uncomment this to  put the arm in sleep position after a sequence
 //PutArmToSleep();
}
