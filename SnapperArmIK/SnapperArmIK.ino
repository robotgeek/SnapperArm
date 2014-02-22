/*
//=============================================================================
 Sources used:
 https://github.com/KurtE
 
 http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 
 Application Note 44 - Controlling a Lynx6 Robotic Arm 
 http://www.micromegacorp.com/appnotes.html
 http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf
 
 
 //  This code is a Work In Progress and is distributed in the hope that it will be useful,
 //  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 //  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 //  
 //=============================================================================
 
 Sources used:
 http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 
 Application Note 44 - Controlling a Lynx6 Robotic Arm 
 http://www.micromegacorp.com/appnotes.html
 http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf
 
 */

#include <ServoEx.h>
#include "InputControl.h"

ServoEx    ArmServo[5];

boolean looping = true;

void setup(){
  // Attach servo and set limits
  ArmServo[BAS_SERVO].attach(3, BASE_MIN, BASE_MAX);
  ArmServo[SHL_SERVO].attach(5, SHOULDER_MIN, SHOULDER_MAX);
  ArmServo[ELB_SERVO].attach(6, ELBOW_MIN, ELBOW_MAX);
  ArmServo[WRI_SERVO].attach(9, WRIST_MIN, WRIST_MAX);
  ArmServo[GRI_SERVO].attach(10, GRIPPER_MIN, GRIPPER_MAX);
  // send arm to default X,Y,Z coord
  doArmIK(true, g_sIKX,g_sIKY,g_sIKZ,g_sIKGA);
  SetServo(sDeltaTime);
  // send arm to center position, used for error adjustment


  // start serial
  Serial.begin(9600);
  Serial.println("Starting RobotGeek Analog IK Demo");
  delay(500);
  AnalogControlLoop();
}


void loop(){

  int inByte = Serial.read();

  switch (inByte) {

  case '1':    
    AnalogControlLoop();
    break;    

  case '2':
    SequenceLoop(); 
    break;


  }

}



void MenuOptions(){

  Serial.println("###########################"); 
  Serial.println("Please enter option 1-2.");     
  Serial.println("1) Start Analog IK Control Mode");        
  Serial.println("2) Start Preprogrammed Sequence Mode");     
  Serial.println("###########################"); 
}



void AnalogControlLoop(){
  Serial.println("Analog IK Control Mode Active. Send '1' to exit"); 
  do
  {
    //Process analog input from ArmControl, translate to working X,Y,Z,GA Coord
    ProcessAnalogInput3D();
    //Calculate goal positions of servos based on X,Y,Z,GA coord determined by ProcessUserInput3D()
    doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 
    //Set servo positions via sDeltaTime interpolation value (set in UserInput as well)
    SetServo(0);
  } 
  while(Serial.available() == 0); 
  Serial.read(); // Read & discard the character that got us out of the loop.
  //while (Serial.read() != 'stop');
  Serial.println("Exiting Analog IK Control Mode. Current Values:"); 
  Serial.println(g_sIKX, 2);
  Serial.println(g_sIKY, 2);
  Serial.println(g_sIKZ, 2);
  Serial.println(g_sIKGA, 2);
  Serial.println(Gripper, DEC);
  delay(500);
  MenuOptions();
}

void SequenceLoop(){
  Serial.println("Sequencing Mode Active. Send '1' to exit"); 
  do
  {
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
  //SequencingControl(X-axis, Y-Axis, Z-axis, Wrist Angle, Gripper, Interpolation, Delay)   
    SequencingControl(0, 150, 150, 0, 1500, 2000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 2
    //###########################################################//
    SequencingControl(-100, 150, 150, 0, 1500, 1000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 3
    //###########################################################//
    SequencingControl(0, 100, 100, 0, 1500, 3000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 4
    //###########################################################//
    SequencingControl(100, 150, 200, 0, 1500, 1000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 5
    //###########################################################//
    SequencingControl(0, 100, 100, -15, 1500, 2000, 1000);
    //###########################################################//

  } 
  while(Serial.available() == 0); 
  Serial.read(); // Read & discard the character that got us out of the loop.
  //while (Serial.read() != 'stop');
  Serial.println("Exiting Sequencing Mode."); 
  MenuOptions();
}


void SequencingControl(float X, float Y, float Z, float GA, int grip, int interpolate, int pause){
    doArmIK(true, X, Y, Z, GA); 
    Gripper = grip;
    SetServo(interpolate);
    delay(interpolate + pause);
}
  

