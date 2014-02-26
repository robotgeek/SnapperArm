/***********************************************************************************
 *  }--\     RobotGeek Snapper Robotic Arm     /--{
 *      |     Analog Backhoe Control Code     |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will move each joint of the arm based on analog inputs.
 *
 *  Snapper Arm Getting Started Guide
 *  http://learn.trossenrobotics.com/33-robotgeek-getting-started-guides/robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide
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
 *    Use an external power supply and set both PWM jumpers to 'VIN'
 *
 *  CONTROL
 *    Turn the 
 *
 *
 *  NOTES
 *    ANALOG INPUT MAPPING
 *      This code uses a combination of direct and incremental code for converting 
 *      analog inputs into servo positions
 *    
 *      Direct/Absolute
 *        Absolute positioning is used for the knobs controlling the base and gripper servo.
 *        This means that the value of the knob is mapped directly to the corresponding servo
 *        value. This method is ideal for sensors that stay at static positions such as
 *        knobs and sliders. 
 *    
 *      Incremental
 *        Incremental code is used for the joysticks controlling the shoulder, elbow and
 *        gripper servo. Each joystick value is mapped to a small realtiveley small positive
 *        or negative value. This value is then added to the currrent position of the servo.
 *        The action of slowly moving the joystick away from its center position can slowly 
 *        move each joint of the robot. When the joystick is centered, no movement is made
 *     
 *      The choice for using Direct/Incremental mapping for each joint was made based
 *      on usability, however the code can be modified so that any joint can use
 *      either direct or incremental mapping
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
 ***********************************************************************************/
#include <ServoEx.h>
#include "InputControl.h"

ServoEx    ArmServo[5];


// Original v1.0 Arm using Rotation Knob/Absolute control for Base input. Uncomment next line to use v1.0 code (v11 define must be commented out).
//#define v10

// Revised v1.1 Arm using Incremental Joystick Lever for Base Input. Uncomment next line to use v1.1 code (v10 define must be commented out)
#define v11



//===================================================================================================
// Setup 
//====================================================================================================
void setup() 
{
  // Attach servo and set limits
  ArmServo[BAS_SERVO].attach(3, BASE_MIN, BASE_MAX);
  ArmServo[SHL_SERVO].attach(5, SHOULDER_MIN, SHOULDER_MAX);
  ArmServo[ELB_SERVO].attach(6, ELBOW_MIN, ELBOW_MAX);
  ArmServo[WRI_SERVO].attach(9, WRIST_MIN, WRIST_MAX);
  ArmServo[GRI_SERVO].attach(10, GRIPPER_MIN, GRIPPER_MAX);


  // start serial
  Serial.begin(9600);
  Serial.println("Starting RobotGeek Analog BackHoe Demo");
  delay(500);
  
  SetServo(sDeltaTime);  // Move servos to defualt positions
  BackhoeLoop();
}



void loop(){

  int inByte = Serial.read();

  switch (inByte) {

  case '1':    
    BackhoeLoop();
    break;    

  case '2':
    SequenceLoop(); 
    break;

  }
}


void MenuOptions(){

  Serial.println("###########################"); 
  Serial.println("Please enter option 1-2.");     
  Serial.println("1) Start Backhoe Control Mode");        
  Serial.println("2) Start Preprogrammed Sequence Mode");     
  Serial.println("###########################"); 
}



void BackhoeLoop(){
  Serial.println("Backhoe Control Mode Active. Send '1' to exit"); 
  do
  {
    //Process analog input from 
    ProcessAnalogBackhoe();

  } 
  while(Serial.available() == 0); 
  Serial.read(); // Read & discard the character that got us out of the loop.
  //while (Serial.read() != 'stop');
  delay(100);
  Serial.println("Exiting Backhoe Control Mode."); 
  Serial.println("Current Joint Position Values:");
  Serial.print("Base Joint: ");
  Serial.println(Base, DEC);
  Serial.print("Shoulder Joint: ");
  Serial.println(Shoulder, DEC);
  Serial.print("Elbow Joint: ");
  Serial.println(Elbow, DEC);
  Serial.print("Wrist Joint: ");
  Serial.println(Wrist, DEC);
  Serial.print("Gripper Joint: ");
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
    //SequencingControl(Base, Shoulder, Elbow, Wrist, Gripper, Interpolation, Delay)
    Serial.println("Moving to Sequence Position 1");   
    BackhoeSequencingControl(1500, 1500, 1500, 1500, 1500, 2000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 2
    //###########################################################//
    Serial.println("Moving to Sequence Position 2");  
    BackhoeSequencingControl(1800, 1500, 1500, 1500, 1500, 2000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 3
    //###########################################################//
    Serial.println("Moving to Sequence Position 3");  
    BackhoeSequencingControl(1200, 1500, 1700, 1500, 1500, 2000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 4
    //###########################################################//
    Serial.println("Moving to Sequence Position 4");  
    BackhoeSequencingControl(1800, 1500, 1500, 1500, 1500, 2000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 5
    //###########################################################//
    Serial.println("Moving to Sequence Position 5");  
    BackhoeSequencingControl(1200, 1500, 1200, 1500, 1500, 2000, 1000);
    //###########################################################//

  } 
  while(Serial.available() == 0); 
  Serial.read(); // Read & discard the character that got us out of the loop.
  //while (Serial.read() != 'stop');
  delay(100);
  Serial.println("Pausing Sequencing Mode."); 
  delay(500);
  MenuOptions();
}


void BackhoeSequencingControl(int base, int shoulder, int elbow, int wrist, int grip, int interpolate, int pause){
  Base = base;
  Shoulder = shoulder;
  Elbow = elbow;
  Wrist = wrist;
  Gripper = grip;
  SetServo(interpolate);
  delay(interpolate + pause);
}
