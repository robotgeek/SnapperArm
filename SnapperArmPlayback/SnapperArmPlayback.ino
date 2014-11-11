/***********************************************************************************
 *  }--\     RobotGeek Snapper Robotic Arm     /--{
 *      |       Analog IK Control Code        |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will move the arm to an X/Y/Z coordinate based on the inputs
 *  from the analog inputs (joysticks and knob). This sketch can also be used to play
 *  back a pre-programmed sequence.
 *
 *  Snapper Arm Getting Started Guide
 *   http://learn.robotgeek.com/getting-started/33-robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide.html
 *  Using the IK Firmware
 *    http://learn.robotgeek.com/demo-code/demo-code/154-robotgeek-snapper-joystick-inverse-kinematics-demo.html
 *
 *
 *  WIRING
 *    Servos
 *      Digital I/O 3 - Base Rotation - Robot Geek Servo 
 *      Digital I/O 5 - Shoulder Joint - Robot Geek Servo 
 *      Digital I/O 6 - Elbow Joint - Robot Geek Servo 
 *      Digital I/O 9 - Wrist Joint - Robot Geek Servo 
 *      Digital I/O 10 - Gripper Servo - 9g Servo 
 *
 *    Analog Inputs
 *      Analog 0 - Joystick (Horizontal)
 *      Analog 1 - Joystick (Vertical)
 *      Analog 2 - Joystick (Vertical)
 *      Analog 3 - Joystick (Vertical)
 *      Analog 4 - Rotation Knob 
 *      
 *    Digital Inputs
 *      Digital 2 - Button 1
 *      Digital 4 - Button 2
 *
 *  
 
 *
 *
 *  NOTES
 *
 *    SERVO POSITIONS
 *      The servos' positions will be tracked in microseconds, and written to the servos
 *      using .writeMicroseconds()
 *        http://arduino.cc/en/Reference/ServoWriteMicroseconds
 *      For RobotGeek servos, 600ms corresponds to fully counter-clockwise while
 *      2400ms corresponds to fully clock-wise. 1500ms represents the servo being centered 
 *
 *      For the 9g servo, 900ms corresponds to fully counter-clockwise while
 *      2100ms corresponds to fully clock-wise. 1500ms represents the servo being centered 
 *
 *
 *  This code is a Work In Progress and is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 * Sources used:
 * https://github.com/KurtE
 * 
 * http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 * 
 * Application Note 44 - Controlling a Lynx6 Robotic Arm 
 * http://www.micromegacorp.com/appnotes.html
 * http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf
 * 
 * 
 *   This code is a Work In Progress and is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 *   
 ***********************************************************************************/

#include <ServoEx.h>

//kinematics file - local
#include "Kinematics.h"

//armSequence
#include "armSequence.h"

//set USE_BUTTON to FALSE if you do not have a button attached to the ArbotiX-M
#define USE_BUTTON true

//set BUTTON TRUE to HIGH for a button with built in pullup resistor like the RobotGeek Pushbutton.
//set BUTTON TRUE to LOW for a simple 2-pin pushbutton.
#define BUTTON_TRUE HIGH
#define BUTTON1_PIN 2



int buttonState1;         

ServoEx    ArmServo[5];

//===================================================================================================
// Setup 
//===================================================================================================
void setup(){
  // Attach servo and set limits
  ArmServo[BAS_SERVO].attach(3, BASE_MIN, BASE_MAX);
  ArmServo[SHL_SERVO].attach(5, SHOULDER_MIN, SHOULDER_MAX);
  ArmServo[ELB_SERVO].attach(6, ELBOW_MIN, ELBOW_MAX);
  ArmServo[WRI_SERVO].attach(9, WRIST_MIN, WRIST_MAX);
  ArmServo[GRI_SERVO].attach(10, GRIPPER_MIN, GRIPPER_MAX);

  // initialize pin 2 on interupt 0
  attachInterrupt(0, stateChange, CHANGE);  
  // initialize the pins for the pushbutton as inputs:  


  pinMode(BUTTON1_PIN, INPUT);     
     


  // send arm to default X,Y,Z coord
  doArmIK(true, g_sIKX,g_sIKY,g_sIKZ,g_sIKGA);
  SetServo(sDeltaTime);

  // start serial
  Serial.begin(9600);
  Serial.println("Starting RobotGeek Analog IK Demo");
  delay(500);
  
  
}


void loop(){

  //If USE_BUTTON is TRUE then read from the button pin
  if(USE_BUTTON == true)
  { 
    //use digitalRead to store the current state of the pushbutton in one of the 'buttonState' variables
    buttonState1 = digitalRead(BUTTON1_PIN);
    
    //if the button has been pushed, run the sequence
    //the sequence will run until the button is pressed again.
    if (buttonState1 == BUTTON_TRUE) 
    {     
      playSequence();
    } 
  }

  
  //read a byte from the serial port
  int inByte = Serial.read();

  //if the character is '1' , run the sequence
  if (inByte == '1') 
  {     
    playSequence();
  } 
  
  
  
}






//===================================================================================================
// SetServo: Writes Servo Position Solutions
//===================================================================================================
void SetServo(unsigned int DeltaTime)
{
  ServoGroupMove.start();
  ArmServo[BAS_SERVO].writeMicroseconds(Base + BAS_SERVO_ERROR);
  ArmServo[SHL_SERVO].writeMicroseconds(Shoulder + SHL_SERVO_ERROR);
  ArmServo[ELB_SERVO].writeMicroseconds(Elbow + ELB_SERVO_ERROR);
  ArmServo[WRI_SERVO].writeMicroseconds(Wrist + WRI_SERVO_ERROR);
  ArmServo[GRI_SERVO].writeMicroseconds(Gripper + GRI_SERVO_ERROR);
  ServoGroupMove.commit(DeltaTime);
}



void IKSequencingControl(float X, float Y, float Z, float GA, int grip, int interpolate, int pause, int enable ){
  if(enable == 1)
  {
    doArmIK(true, X, Y, Z, GA); 
    Gripper = grip;
    SetServo(interpolate);
    delay(interpolate + pause);
  }
}


void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable ){
  if(enable == 1)
  {
    doArmIK(true, X, Y, Z, GA); 
    Gripper = grip;
    SetServo(interpolate);
    delay(interpolate + pause);
  }
}


void stateChange()
{
  if(playState == 1)
    {
      playState = 0;
    }
    
}

