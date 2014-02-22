/***********************************************************************************
 *  }--\     RobotGeek Snapper Robotic Arm     /--{
 *      |         Analog Control Code         |
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
#include <Servo.h>

// Original v1.0 Arm using Rotation Knob/Absolute control for Base input. Uncomment next line to use v1.0 code (v11 define must be commented out).
//#define v10

// Revised v1.1 Arm using Incremental Joystick Lever for Base Input. Uncomment next line to use v1.1 code (v10 define must be commented out)
#define v11

// enables serial debugging via serial monitor @ 38400 baud. Outputs actual uS servo positions, including any offsets.
//#define SERIAL_DEBUG

//define analog pins that will be connected to the joystick pins
#define BASE     0  //connected to Horizontal Axis on Joystick # 1
#define SHOULDER 1  //connected to Vertical Axis on Joystick # 2
#define ELBOW    2  //connected to Vertical Axis on Joystick # 3
#define WRIST    3  //connected to Vertical Axis on Joystick # 4
#define GRIPPER  4  //connected to Rotation Knob / Potentiometer # 1

//generic deadband limits - not all joystics will center at 512, so these limits remove 'drift' from joysticks that are off-center.
#define DEADBANDLOW 482   //decrease this value if drift occurs, increase it to increase sensitivity around the center position
#define DEADBANDHIGH 542  //increase this value if drift occurs, decrease it to increase sensitivity around the center position

// SERVO CONFIG  //
//
// Declare servo objects
Servo BAS_SERVO;    //base servo - RobotGeek Servo
Servo SHL_SERVO;    //shoulder servo - RobotGeek Servo 
Servo ELB_SERVO;    //elbow servo - RobotGeek Servo 
Servo WRI_SERVO;    //wrist servo - RobotGeek Servo
Servo WRO_SERVO;    //wrist rotate servo - RobotGeek Servo (unused for snapper arm)        
Servo GRI_SERVO;    //gripper servo - 9g servo

// Servo position limitations - limits in microseconds
#define BASE_MIN      600     //full counterclockwise for RobotGeek 180 degree servo
#define BASE_MAX      2400    //full clockwise for RobotGeek 180 degree servo
#define SHOULDER_MIN  600
#define SHOULDER_MAX  2400
#define ELBOW_MIN     600
#define ELBOW_MAX     2400
#define WRIST_MIN     600
#define WRIST_MAX     2400 
#define GRIPPER_MIN   900    //full counterclockwise for 9g servo
#define GRIPPER_MAX   2100   //full clockwise for 9g servo

// Define servo offsets in +/- uS. Adjust if your arm is not centering properly.
#define BAS_SERVO_ERROR 0 //(+ is CW, - is CCW)
#define SHL_SERVO_ERROR 0 //(+ is forward, - is backward)
#define ELB_SERVO_ERROR 0 //(+ is up, - is down)
#define WRI_SERVO_ERROR 0 //(+ is up, - is down)
#define GRI_SERVO_ERROR 0 //(+ is tighten grip, - is loosen grip) 

//present positions of the servos 
int Base     =1500;    //holds the present position of the Base servo, starts at 1500 (centered)
int Shoulder =1500;    //holds the present position of the Shoulder servo, starts at 1500 (centered)
int Elbow    =1500;    //holds the present position of the Elbow servo, starts at 1500 (centered)
int Wrist    =1500;    //holds the present position of the wrist servo, starts at 1500 (centered)
int Gripper  =1500;    //holds the present position of the gripper servo, starts at 1500 (centered)


//last read values of analog sensors (Native values, 0-1023)
int joyBaseVal = 0;     //present value of the base rotation knob (analog 0)
int joyShoulderVal = 0; //present value of the shoulder joystick (analog 1)
int joyElbowVal = 0;    //present value of the elbow joystick (analog 2)
int joyWristVal = 0;    //present value of the wrist joystick (analog 3)
int joyGripperVal = 0;  //present value of the gripper rotation knob (analog 4)

//last calculated values of analog sensors (Mapped values)
//knob values (base and gripper) will be mapped directly to the servo limits
//joystick values (shoulder, elbow and wrist) will be mapped from -speed to speed, to faciliate incremental control
int joyBaseMapped = 0;      //base knob value, mapped from 1-1023 to BASE_MIN-BASE_MAX
int joyShoulderMapped = 0;  //shoulder joystick value, mapped from 1-1023 to -speed to speed
int joyElbowMapped = 0;     //elbow joystick value, mapped from 1-1023 to -speed to speed
int joyWristMapped = 0;     //wrist joystick value, mapped from 1-1023 to -speed to speed
int joyGripperMapped = 0;   //gripper knob  value, mapped from 1-1023 to GRIPPER_MIN-GRIPPER_MAX

int speed = 10;  //speed modififer, increase this to increase the speed of the movement

int serialtimer = 0;


//===================================================================================================
// Setup 
//====================================================================================================
void setup() 
{
  Serial.begin(38400);
  // Attach servo and set limits
  BAS_SERVO.attach(3, BASE_MIN, BASE_MAX);
  SHL_SERVO.attach(5, SHOULDER_MIN, SHOULDER_MAX);
  ELB_SERVO.attach(6, ELBOW_MIN, ELBOW_MAX);
  WRI_SERVO.attach(9, WRIST_MIN, WRIST_MAX);
  GRI_SERVO.attach(10, GRIPPER_MIN, GRIPPER_MAX);
  
  delay(1000);  //wait 1 second
  
  set_servo();  // Move servos to defualt positions
}

void loop() 
{
  
   //read analog values from analog sensors
   joyBaseVal = analogRead(BASE);
   joyShoulderVal = analogRead(SHOULDER);
   joyElbowVal = analogRead(ELBOW);
   joyWristVal = analogRead(WRIST);
   joyGripperVal = analogRead(GRIPPER);
    
// Base joint is handled differently depending upon analog control solution. v1.0 Snapper Arms used a rotational knob with direct/absolute control of the base servo, where v1.1 Snapper Arms use an incremental Joystick.    
        
#ifdef v10   
   joyBaseMapped = map(joyBaseVal, 1023, 0, BASE_MIN, BASE_MAX);  //Mapping analog knob value to servo PWM signal range
   Base = joyBaseMapped; //set servo position variable to the mapped value from the knob. Absolute/direct control.
#endif

#ifdef v11   
   //only update the base joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyBaseVal > DEADBANDHIGH || joyBaseVal < DEADBANDLOW)
   {
     joyBaseMapped = map(joyBaseVal, 0, 1023, -speed, speed); //Map analog value from native joystick value (0 to 1023) to incremental change (-speed to speed)
     Base += joyBaseMapped; //add mapped base joystick value to present Base Value (positive values of joyBaseMapped will increase the position, negative values will decrease the position)
   }
#endif


   //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyShoulderVal > DEADBANDHIGH || joyShoulderVal < DEADBANDLOW)
   {
     joyShoulderMapped = map(joyShoulderVal, 0, 1023, -speed, speed); //Map analog value from native joystick value (0 to 1023) to incremental change (-speed to speed)
     Shoulder = Shoulder - joyShoulderMapped; //add mapped shoulder joystick value to present Shoulder Value (positive values of joyShoulderMapped will increase the position, negative values will decrease the position)
   }

   //only update the elbow joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyElbowVal > DEADBANDHIGH || joyElbowVal < DEADBANDLOW)
   {
     joyElbowMapped = map(joyElbowVal, 0, 1023, -speed, speed); //Map analog value from native joystick value (0 to 1023) to incremental change (-speed to speed)
     Elbow = Elbow + joyElbowMapped;//add mapped elbow joystick value to present elbow Value (positive values of joyElbowMapped will increase the position, negative values will decrease the position)
   }
   

   //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyWristVal > DEADBANDHIGH || joyWristVal < DEADBANDLOW)
   {
     joyWristMapped = map(joyWristVal, 0, 1023, -speed, speed); //Map analog value from native joystick value (0 to 1023) to incremental change (-speed to speed)
     Wrist = Wrist + joyWristMapped;//add mapped wrist joystick value to present wrist Value (positive values of joyWristMapped will increase the position, negative values will decrease the position)
   }
   
   
   //Mapping analog joystick value to servo PWM signal range
   joyGripperMapped = map(joyGripperVal, 0, 1023, GRIPPER_MIN, GRIPPER_MAX);
   Gripper = joyGripperMapped;//set servo position variable to the mapped value from the knob

      
    //enforce upper/lower limits for servo position variable 
    //(actual servo positions min/max are setup above as a part of .attach(), this is just for variable)
    if (Base < BASE_MIN)
    {
      Base =BASE_MIN;
    }  
    else if (Base > BASE_MAX)
    {
      Base =BASE_MAX;
    }
    
    //enforce upper/lower limits for shoulder servo
    if (Shoulder < SHOULDER_MIN)
    {
      Shoulder =SHOULDER_MIN;
    }  
  
    else if (Shoulder > SHOULDER_MAX)
    {
      Shoulder =SHOULDER_MAX;
    }
  
  
  
    //enforce upper/lower limits for elbow servo
    if (Elbow < ELBOW_MIN)
    {
      Elbow = ELBOW_MIN;
    }  
    else if (Elbow > ELBOW_MAX)
    {
      Elbow = ELBOW_MAX;
    }
    
    
    //enforce upper/lower limits for wrist servo
    if (Wrist < WRIST_MIN)
    {
      Wrist = WRIST_MIN;
    }  
  
    else if (Wrist > WRIST_MAX)
    {
      Wrist =WRIST_MAX;
    }
  
  
    //enforce upper/lower limits for gripper servo
    if (Gripper < GRIPPER_MIN)
    {
      Gripper =GRIPPER_MIN;
    }  
    else if (Gripper > GRIPPER_MAX)
    {
      Gripper = GRIPPER_MAX;
    }
    
    //Funciton to set PWM Servo positions
    set_servo();
    
#ifdef SERIAL_DEBUG
    serialprintout();
    serialtimer++;
#endif    
  }


/******************************************************
 *  set_servo()
 *
 *  This function sets the 5 servos on the snapper arm
 *  to the 5 positions variables. All servos and
 *  positions variables are globals 
 *
 *  Parameters:
 *    none
  *
 *  Globals Used:
 *      Servo BAS_SERVO
 *      Servo WRI_SERVO
 *      Servo SHL_SERVO
 *      Servo ELB_SERVO
 *      Servo GRI_SERVO
 *      int Base
 *      int Wrist
 *      int Shoulder
 *      int Elbow
 *      int Gripper
 *
 *  Returns: 
 *    none
 ******************************************************/ 

void set_servo()
{
  BAS_SERVO.writeMicroseconds(Base + BAS_SERVO_ERROR);
  SHL_SERVO.writeMicroseconds(Shoulder + SHL_SERVO_ERROR);
  ELB_SERVO.writeMicroseconds(Elbow + ELB_SERVO_ERROR);
  WRI_SERVO.writeMicroseconds(Wrist + WRI_SERVO_ERROR);
  GRI_SERVO.writeMicroseconds(Gripper + GRI_SERVO_ERROR);
    delay(10);
}

void serialprintout()
{
  if (serialtimer == 50)
    {
      serialtimer = 0;
      Serial.println("#####################");
      Serial.print("Base Value: ");
      Serial.println(Base + BAS_SERVO_ERROR);
      Serial.print("Shoulder Value: ");
      Serial.println(Shoulder + SHL_SERVO_ERROR);
      Serial.print("Elbow Value: ");
      Serial.println(Elbow + ELB_SERVO_ERROR);
      Serial.print("Wrist Value: ");
      Serial.println(Wrist + WRI_SERVO_ERROR);
      Serial.print("Gripper Value: ");
      Serial.println(Gripper + GRI_SERVO_ERROR);
    }
}


