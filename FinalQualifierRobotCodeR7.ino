/*
 * Names: Drew Blessed, Alex Kaiser, Eric Spiddle
 * Date: 11-23-19
 * Input Sources: 3 IR sensors - front, left, and right side of robot
 *                2 click swithches at front - left/right of robot
 *                1 photodiode sensor in ball holding area
 *                1 button on top of the robot
 * Inputs: IR distance readings, click switch on/off, photdiode on/off, button on/off
 * Output Sources: Serial monitor, motor, servo
 * Outputs: IR distance readings and other diagnostics, motor direction & speed, servo direction & speed
 * Description: 
 */



#include <Wire.h> // library for motorshield 
#include <Adafruit_MotorShield.h> // library for motorshield 
#include "utility/Adafruit_MS_PWMServoDriver.h" // library for motorshield 
#include<Servo.h> // library for servo 

#define KP 1 // macro for kp 1 pid  last - 1
#define KD 8 // macro for kd 1 pid  last - 4

#define KP2 1 // macro for kp2 pid  last - 1
#define KD2 8//macro for kd2 pid   last - 5

#define IRMAX 200 // macro for setting max IR value
#define SWITCHR 2 // limit switch pin macro 
#define SWITCHL 3 // limit switch pin macro 

Servo PaintRoller; // servo variable for paintroller
const int trackerPin = 9; // pin for photodiode
const int buttonPin = 13; //pin attached to button
int buttonState = 0;      //decides if button is pushed
const int IR_Sense_Front = A2;     //anlog pin for front IR
const int IR_Sense_Left = A0;      //anlog pin for left IR
const int IR_Sense_Right = A1;     //anlog pin for right IR
int IrVoltFront = 0;              //voltage from front IR
int IrVoltLeft = 0;             //voltage from left IR
int IrVoltRight = 0;            //voltage from right IR
int cmFront = 0;                //distance from front IR
int origcmFront = 0;            //original distance from front for comparison
int cmLeft = 0;                 //distance from left IR sensor
int origcmLeft = 0;             //original distance from left for comparison
int cmRight = 0;                //distance from right IR sensor
int origcmRight = 0;            //original distance from right for coomparison
int isBall = 0;                // integer to check wether robot has ball or not
int leftCmHolder;
int rightCmHolder;
int frontCmStuck;
int firstTimeThrough = 1;     // has robot turn towards ball if it is the first time through the triangle func

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //motorshield object allows for use of motorshield
const int LeftMaxSpeed = 50;                          //sets max speed for left motor
const int RightMaxSpeed = 55;                         //sets max speed for right motor
const int LeftMinMotorSpeed = 25;                     // sets minimum speed for left motor
const int RightMinMotorSpeed = 40;                    // sets minimum speed for right motor
const int rightTriangleFunc = 60;
const int leftTriangleFunc = 45;

Adafruit_DCMotor *RightMotor = AFMS.getMotor(4);   //assigns  right motor to port 4 on shield
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);    //assigns left motor to port 1 on shield

int RightMotorSpeed = 35;              //variable to hold right motor speed
int LeftMotorSpeed = 25;            //variable to hold left motor speed
int RightMotorSpeed1 = 40;  
int LeftMotorSpeed1 = 20;

unsigned long currTime = millis();   //variable to hold time values
unsigned long currTimeLimit;         // variable for limit switch times
unsigned long currTimeLimit1;        // variable for limit switch times
unsigned long currTimeS;


int i = 0;  //for for loop counting
int trackDetect = HIGH; // sets the photodiode state to high intially 

enum states // enumeration for state machine 
{
  Find_Ball, // state for robot to find a ball
  Decide_Wall, // state for robot to decide which wall to go to 
  Follow_Wall, // state for the robot to follow the wall 
  Score_Left, // scoring if following a wall on left 
  Score_Right // scoring if following a wall on right 
};
//enum states roboState = Follow_Wall;  //starts program in state looking for ball
enum states roboState = Find_Ball;

void setup() 
{

  Serial.begin(9600); // intializes Serial monitor 
  AFMS.begin(); // begins the motorshield 
  
  pinMode(trackerPin, INPUT); // sets the tracker pin to an input 
  pinMode(buttonPin, INPUT); // sets the buttonPin to an inpit 
  PaintRoller.attach(10);  // attaches paintroller servo to digital pin 10
  PaintRoller.write(99);
  funcButtonStart(); // runs the button function so robot waits for button press to go
  
  pinMode(SWITCHR, INPUT); // sets the right limit switch pinmode to input 
  pinMode(SWITCHL, INPUT); // sets the left limit switch pinmode to input 

}

void loop()
{
  switch (roboState) { // switch statement for the states of robot 

    case Find_Ball: // case of finding a ball 
      FuncLimitSwitch(); // robot calls the limit switch function to check if robot is stuck
      triangle_Func();

      break;

    case Decide_Wall:
      isBall = 0; // sets the isBall int back to 0 for use in if condition
       
     // FuncLimitSwitch();  // robot calls the limit switch function to check if robot is stuck
      FuncDecideWall(); // function that decides what wall the robot will go to 
      
      FuncGoToWall(); // function that takes robot to the wall 

      break; // breaks so the switch doesn't fall through 

    case Follow_Wall: // case for following a wall 

      FuncDecideWallToFollow(); // function to decide which wall the robot will follow 
      
      break; // breaks so the switch doesn't fall through 


    case Score_Left: // case for scoring on the left wall 
     // Serial.println("We finna score and HERES THE CAR"); // LOL 

      FuncScoreLeft(); //calls on line tracking state
      
      break; // breaks so the switch doesn't fall through 
      
    case Score_Right: // case where goal is to the right side of the robot 
    
      FuncScoreRight(); // function that scores a ball when the goal is on the right side of the robot
      
      break; // breaks so the siwtch doesn't fall through 
      
  }

}
void funcButtonStart()
{ //function to allow robot to allow button to start program
  Serial.println("Press button");
  while (buttonState == 0)          //infinite loop to not run until button is pressed
  {
    buttonState = digitalRead(buttonPin);   //takes button pressed as input
  }
  Serial.println("Button pressed");
}
void funcIR_Dist_Sense()         //function to find distances around robot
{
  IrVoltFront = analogRead(IR_Sense_Front);     //gets voltage and calculates cm for front
  cmFront = (6787 / (IrVoltFront - 3) ) - 4;    //equation to calculate the distance read by the front IR sensor

  if (cmFront < 0 || cmFront > IRMAX)        //reassigns IR value to max value if negative or too large
  {
    cmFront = IRMAX;                     //reassigns IR value to max value
  }
  Serial.print("Front: ");             //prints front IR sensor reading diagnostic
  Serial.println(cmFront);            

  IrVoltLeft = analogRead(IR_Sense_Left);       //gets voltage and calculates cm for left
  cmLeft = (6787 / ( IrVoltLeft - 3) ) - 4;     //equation to calculate distance read by Left IR sensor

  if (cmLeft < 0 || cmLeft > IRMAX)            //checks if left IR reading is negative or extremely high
  {
    cmLeft = IRMAX;                       //reassings left IR sensor reading to max value if IF statement is true
  }
  Serial.print("Left: ");              //prints out left IR sensor value
  Serial.println(cmLeft);

  IrVoltRight = analogRead(IR_Sense_Right);      //gets voltage and calculates cm for right
  cmRight = (6787 / (IrVoltRight - 3) ) - 4;    //equation to calculate distance read by right IR sensor

  if (cmRight < 0 || cmRight > IRMAX)    //checks if right IR sensor reading is negative or extremely high
  {
    cmRight = IRMAX;              //reassigns right IR sensor reading to max value if IF statement is true
  }
  Serial.print("Right: ");       //prints out right IR sensor reading
  Serial.println(cmRight);     
  Serial.println("");         //prints out gap line for easier reading
  
}
void triangle_Func()            //function to drive the robot around the arena
{
  FuncLimitSwitch();             //calls limit switch to check if robot hit an obstacle
  Serial.println("I am in the triangle function");  //diagnostic printout
  PaintRoller.write(115);                          //sets roller speed to full to pick up a ball
  FuncCheckforBall();                        //calls function that checks for ball 
  isBall = FuncBackupCheck();               //calls function that returns if ball is in holder or not
  if (isBall == 1)                      //if ball is in holder
  {
    
    roboState = Decide_Wall;           //calls function that decides which side wall is on
    Serial.println("I have ball");
  }
  else if (isBall == 0)            //if there is no ball in holder
  {
    PaintRoller.write(115);
    roboState = Find_Ball;             //continues searching for ball
    Serial.println("Obstacle");      //ball reasing was actually an obstacle
  } 
}
void FuncCheckforBall()                //function that checks if ball is in holder
{
  trackDetect = digitalRead(trackerPin); // checks to see if robot has ball
  while (trackDetect == HIGH ) // while robot does not have a ball go straight and turn
  {

    trackDetect = digitalRead(trackerPin); // check for a ball while in the while loop
    funcDrive_Straight(); // calls the drive straight function 
    funcIR_Dist_Sense(); // calls the IR function to check location 
    if (cmRight > cmLeft) // if right has more room
    {
      FuncTurnRight(); // turn right 
    }
    else if (cmRight <= cmLeft) // if left has more room  
    {
      FuncTurnLeft(); // turn left 
    }
    Serial.println("Checking for Ball"); // for debugging, prints checking for ball 
  }
}

void funcDrive_Straight() // function that drives the robot straight  
{

  if (firstTimeThrough == 1) // if this is the first time running through the code
  {
    currTime = millis(); // set currtime equal to millis 
    while (millis() - currTime < 750) // for 750 ms
    {
      RightMotor ->setSpeed(RightMinMotorSpeed); // set right motor to min speed
      LeftMotor -> setSpeed(LeftMinMotorSpeed); // set left motor to min speed
      RightMotor-> run(BACKWARD); // run motor backwards
      LeftMotor-> run(FORWARD); // run motor forwards 
    }
    firstTimeThrough = 0; // set the firstTimeThrough to 0 
  }

  currTime = millis(); // set currTime to millis 

  while (millis() - currTime < 2200) // for 2 seconds        
  {
    FuncLimitSwitch(); // calls function to check limit switches
    RightMotor -> setSpeed(rightTriangleFunc);  // set right motor to normal speed
    LeftMotor -> setSpeed(leftTriangleFunc); // set left motor to normal speed
    RightMotor -> run(FORWARD);  //sets both motors forward
    LeftMotor -> run(FORWARD);
    
  }

}
void FuncTurnLeft() // function to turn left 
{
  Serial.println("Func turn left"); // for debugging prints turn left
  currTime = millis(); // sets curr time to millis
  while (millis() - currTime < 950) // for 850 ms 
  {
    FuncLimitSwitch(); // check limitswitch
    RightMotor -> setSpeed(RightMinMotorSpeed); // set both motors to min speed
    LeftMotor -> setSpeed(LeftMinMotorSpeed);
    RightMotor -> run(FORWARD); // run right motor forward
    LeftMotor -> run(BACKWARD); // run left motor backward
  }

}
void FuncTurnRight() // function to turn right
{

  Serial.println("func turn Right"); // for debugging prints turn right
  currTime = millis(); // sets currTime to millis
  while (millis() - currTime < 950) // for 850 ms
  {
    FuncLimitSwitch(); // checks limit switches 
    RightMotor -> setSpeed(RightMinMotorSpeed); //sets both motors to min speed
    LeftMotor -> setSpeed(LeftMinMotorSpeed);
    RightMotor -> run(BACKWARD); // run right motor backward
    LeftMotor -> run(FORWARD); // run left motor forward
  }
}

int FuncBackupCheck() // function that backs up 
{
  FuncLimitSwitch(); // checks limit switches
  Serial.println("Backing up"); // debugging prints backing up
  currTime = millis(); // set currtime to millis 
  while (millis() - currTime < 1000) // for 1 second
  {
    FuncLimitSwitch(); // checks limit switches
    RightMotor -> run(BACKWARD); // run both motors backward
    LeftMotor -> run(BACKWARD);
    PaintRoller.write(115); // runs the servo at 115 speed
  }
  
  trackDetect = digitalRead(trackerPin); // checkes to see if robot still has ball
  if (trackDetect == LOW) // if it has the ball
  {
    
    return 1; // return true 
  }
  else // otherwise 
  { 
    
    funcIR_Dist_Sense(); // check location
    if (cmRight < cmLeft) // if left has more room
    {
      FuncTurnLeft(); // turn left
    }
    else // otherwise
    {
      FuncTurnRight(); // turn right
    }
    return 0; // return false 
  }

}
void FuncDecideWall() // function to decide which wall to go to 
{
  funcIR_Dist_Sense(); // checks location
  if (cmRight > cmLeft) // if right has more room
  {
    FuncTurnLeftWall(); // turn for the left wall
  }
  else if (cmLeft >= cmRight) // if left has more room
  {
    FuncTurnRightWall(); // turn for the right wall
  }
}
void FuncTurnLeftWall() // function that turns for the left wall
{
  currTime = millis(); // sets currTime to millis
  while (millis() - currTime < 1200) // for 1.2 seconds
  {
    
    funcIR_Dist_Sense(); // checks location
    RightMotor -> setSpeed(RightMinMotorSpeed); // sets both motor speeds to min
    LeftMotor -> setSpeed(LeftMinMotorSpeed);
    RightMotor -> run(FORWARD); // runs right motor forward
    LeftMotor -> run(BACKWARD); // runs left motor backwards
  }

}
void FuncTurnRightWall() // function that turns for the right wall
{
  currTime = millis(); // sets currTime to millis
  while (millis() - currTime < 1200) // for 1.2s
  {
    funcIR_Dist_Sense(); // checks location
    RightMotor -> setSpeed(RightMinMotorSpeed); // sets both motors to min speed
    LeftMotor -> setSpeed(LeftMinMotorSpeed);
    RightMotor -> run(BACKWARD); // right motor run backwards
    LeftMotor -> run(FORWARD); // left motor run forwards
  }

}
void FuncGoToWall() // function that takes the robot to a wall
{
  int foundWall = 0; // variable to check if robot has found wall
  PaintRoller.write(105); // runs paintroller at 105 speed
  currTime = millis(); // currTime set to millis 
  while (foundWall == 0) // if foundwall is false
  {
    FuncLimitSwitch(); // check limit switches
    funcIR_Dist_Sense(); // check location
    frontCmStuck = cmFront; // set variable for getting stuck equal to front cm 
    if (cmFront > 32) // if the front is greater than 32
    {
      FuncLimitSwitch(); // checks limitswitches
      RightMotor -> setSpeed(rightTriangleFunc); // sets both motors speeds to min
      LeftMotor -> setSpeed(leftTriangleFunc);
      RightMotor ->run(FORWARD); // runs both motors forward
      LeftMotor -> run(FORWARD);
    }
    else // if robot near wall
    {
      foundWall = 1; // set foundWall equal to true
    }
   currTime = millis(); // sets currTime to millis 
   while (millis() - currTime < 200); // wait for 450 ms 
   
   funcIR_Dist_Sense(); // check location 
  if(cmFront == frontCmStuck) // if cmFront is equal to the stuck 
  {
    currTime = millis(); // set currTime equal to millis
    while(millis() - currTime < 1000) // for 1 seconds
    {
       RightMotor -> setSpeed(RightMinMotorSpeed); // sets both motors to min speed
      LeftMotor -> setSpeed(LeftMinMotorSpeed);
      RightMotor -> run(BACKWARD); // run both motors backward
      LeftMotor -> run(BACKWARD);
    }
    funcIR_Dist_Sense(); // checks location
    if (cmRight >= cmLeft) // if right has more room 
    {
    currTime = millis(); // set currTime equal to millis 
    while(millis() - currTime < 500) // for 0.5 s
    {
      RightMotor -> run(BACKWARD); // left motor go backwards
      LeftMotor -> run(FORWARD); // right motor go forward
    }
    }
    else // otherwise 
    {
    currTime = millis(); // set currTime equal to millis 
    while(millis() - currTime < 500) // for 0.5 s
    {
      RightMotor -> run(FORWARD); // run right motor forward
      LeftMotor -> run(BACKWARD); // run left motor backward
    }
    }
    
  }
  
  }
  if (foundWall = 1) // if robot has found a wall 
  {
    roboState = Follow_Wall; // robot state set to follow wall 
  }
}
void FuncDecideWallToFollow () // function to decide which wall the robot should follow 
{
  funcIR_Dist_Sense(); // checks location
  if (cmLeft > cmRight) // if left has more room
  {
    WallFollowRight(); // follow a wall on the right 
  }
  else if (cmRight >= cmLeft) // if right has more room
  {
    WallFollowLeft(); // follow a wall left 
  }
}
void WallFollowRight() // follow a wall righ 
{
  int foundGoal = 1; // set foundGoal equal to 1
  int lastError = 0; // set lastError equal to 0 
  funcIR_Dist_Sense(); // check location 
  while (cmFront < 70) // while the front sensor is less than 70 
  {
    funcIR_Dist_Sense(); // checks location 
    RightMotor -> setSpeed(RightMinMotorSpeed); // sets both motor speeds to min 
    LeftMotor -> setSpeed(LeftMinMotorSpeed); 
    RightMotor -> run(FORWARD); // runs the right motor forward
    LeftMotor -> run(BACKWARD); // runs the left motor backward 
  }
  while (foundGoal == 1) // if robot has found a goal 
  {
    funcIR_Dist_Sense(); // checks location 

    int Position = cmRight; // sets position to the right IR reading 
    Serial.println(cmRight); // prints the right reading out for debugging
    int error = 10 - Position; // sets error equal to the 10 - position 
    int motorSpeed = KP * error + KD * (error - lastError); // using PID we determine the motorspeed for each motor
    lastError = error; // sets the error to the last error
    int leftMotorSpeed = LeftMotorSpeed - motorSpeed; // sets the leftMotorSpeed equal to LeftMotorSpeed - pid motorspeed
    int rightMotorSpeed = RightMotorSpeed + motorSpeed; // sets rightMotorSpeed  equal to rightmotorSpeed - pid motorspeed
    SetMotorSpeeds(rightMotorSpeed, leftMotorSpeed); // sets the motor speeds 
    FuncLimitSwitchWall(); // checks limit switch for wall makes shorter back up 
    funcIR_Dist_Sense(); // checks location 
    if (cmFront < 25) // if the front is less than 25 
    {
      func_LeftTurnCorner(); // turn left at a corner
    }
    if (cmRight > 50) // if the right is greater than 50 score to the right 
    {
      foundGoal = 0; // set found goal to 0 
      roboState = Score_Right; // set the roboState state to score right 
    }

  }
}
void WallFollowLeft () // function to wall follow left 
{
  int foundGoal = 1; // set found goal equal to 1
  int lastError = 0; // set last error equal to last error
  funcIR_Dist_Sense(); // checks for location


  while (cmFront < 70) // while the front is less than 70 
  {
    funcIR_Dist_Sense(); // check location 
    RightMotor -> setSpeed(RightMinMotorSpeed); // set both motors to min speed 
    LeftMotor -> setSpeed(LeftMinMotorSpeed);
    RightMotor -> run(BACKWARD); // run right motor backward 
    LeftMotor -> run(FORWARD);// run left motor forward
  }
  while (foundGoal == 1) // if the robot has not a goal 
  {
    funcIR_Dist_Sense(); // check location

    int Position = cmLeft; // set position equal to left reading 
    int error = 10 - Position; // set error equal to 10 - position 
    int motorSpeed = KP2 * error + KD2 * (error - lastError); // uses PID to calculate next motorSpeed 
    lastError = error; // lastError equal to the 
    int leftMotorSpeed = LeftMotorSpeed + motorSpeed; // sets leftmotorspeed equal to original speed + pid motorspeed
    int rightMotorSpeed = RightMotorSpeed - motorSpeed; // sets right motorspeed equal to original speed - pid motorspeed 
    SetMotorSpeeds(rightMotorSpeed, leftMotorSpeed); // sets the motorspeeds 
    FuncLimitSwitchWall(); // checks limitswitch for wall 
    funcIR_Dist_Sense(); // checks location 
    if (cmFront < 25) // check if cmfront less than 25
    {
      func_RightTurnCorner(); // turn right bc in a corner 
    }
    if (cmLeft > 50) // if the left is greater than 50 score left 
    {
      
      foundGoal = 0; // sets foundgoal equal to 0 
      roboState = Score_Left; // sets robot state equal to score left 
    }

  }
}

void SetMotorSpeeds(int rightMotorSpeed1, int leftMotorSpeed1) // function to set the motorspeed takes both motorspeeds from PID as a parameter 
{
  if (rightMotorSpeed1 > RightMaxSpeed) // if the right motorspeed is greater than max 
  {
    rightMotorSpeed1 = RightMaxSpeed; // set the rightmotorspeed equal to the max 
  }
  if (rightMotorSpeed1 < 0) // if the rightmotor speed is less than 0 
  {
    rightMotorSpeed1 = 0; // set the right motorspeed to 0 
  }
  if (leftMotorSpeed1 > LeftMaxSpeed) // if the left motor speed is greater than max speed 
  {
    leftMotorSpeed1 = LeftMaxSpeed; // set left motorspeed equal to leftmax speed 
  }
  if (leftMotorSpeed1 < 0) // if  left motorspeed is less than 0 
  {
    leftMotorSpeed1 = 0; // set left motor speed to 0 
  }
  RightMotor -> setSpeed(rightMotorSpeed1); // sets both motors to new speed 
  LeftMotor -> setSpeed(leftMotorSpeed1);
  Serial.println(rightMotorSpeed1); // prints out the rightmotorspeed1 for debugging 
  Serial.println(leftMotorSpeed1); // prints out the leftmotorspeed1 for debugging 
  Serial.println(" ");// print a space 
  RightMotor -> run(FORWARD); // runs both motors forward
  LeftMotor -> run(FORWARD);
}
void func_LeftTurnCorner() // turn left corner 
{
  funcIR_Dist_Sense(); // checks location 
  while (cmFront < 60) // if front reading is less than 60 
  {

    funcIR_Dist_Sense(); // checks location
    RightMotor -> setSpeed(RightMinMotorSpeed);  //sets both motors to normal speed
    LeftMotor -> setSpeed(LeftMinMotorSpeed);
    RightMotor-> run(FORWARD); // runs right motor forward 
    LeftMotor -> run(BACKWARD); // runs left motor backward 
  }
}
void func_RightTurnCorner() // function that turns for the right corners
{
  funcIR_Dist_Sense(); // checks location
  while (cmFront < 60) // if front reading is less than 60
  {

    funcIR_Dist_Sense(); // checks location 
    RightMotor -> setSpeed(RightMinMotorSpeed);  //sets both motors to min speed
    LeftMotor -> setSpeed(LeftMinMotorSpeed);
    RightMotor-> run(BACKWARD); // turns right 
    LeftMotor -> run(FORWARD);
  }
}


void FuncScoreLeft() // scores on the left 
{

  funcIR_Dist_Sense(); // checks location 
  while (cmFront > 30) // if the front reading is greater than 30 
  {

    funcIR_Dist_Sense(); // checks location 
    RightMotor -> setSpeed(RightMinMotorSpeed); // set the right motor to min speed 
    RightMotor -> run(FORWARD); // set the right motor to run forward 
    LeftMotor-> setSpeed(LeftMinMotorSpeed); // set the left motor to min speed 
    LeftMotor -> run(BACKWARD); // run the left motor backwards 
 
  }

  //RightMotor -> setSpeed(0);
  //LeftMotor -> setSpeed(0);
  funcIR_Dist_Sense(); // checks location

  while (cmFront < 50) // if the front ir reads less than 50
  {

    funcIR_Dist_Sense(); // checks location 
    RightMotor -> setSpeed(RightMinMotorSpeed); // set both otors speed to min 
    LeftMotor-> setSpeed(LeftMinMotorSpeed);
    RightMotor -> run(FORWARD); // turn left 
    LeftMotor -> run(BACKWARD);
    currTime = millis(); // set currTime equal to millis 
    while(millis() - currTime < 200); //delays for 200 ms
  }
  FuncScore(); // calls robot scoring function 
}


void FuncScoreRight() // function to score the robot on the right 
{

  funcIR_Dist_Sense(); //checks location
  while (cmFront > 30) // while the front reading is greater than 30 
  {
    funcIR_Dist_Sense(); // checks location 
    RightMotor -> setSpeed(RightMinMotorSpeed); // set speed to right min speed
    RightMotor -> run(BACKWARD); // run the rightmotor backwards 
    LeftMotor-> setSpeed(LeftMinMotorSpeed); // set speed to left min speed 
    LeftMotor -> run(FORWARD); // run left motor forward 
  
  }
  funcIR_Dist_Sense(); // checks location 

  while (cmFront  < 50) // while the front is less than 50 
  {
    funcIR_Dist_Sense(); // check location 
    RightMotor -> setSpeed(RightMinMotorSpeed); // set right motor speed to min speed 
    RightMotor -> run(BACKWARD); // run the right motor backwards 
    LeftMotor-> setSpeed(LeftMinMotorSpeed); // set left motor speed to min speed 
    LeftMotor -> run(FORWARD); // run the left motor forward 
    currTime = millis(); // set currTime to millis 
    while(millis() - currTime < 200); // wait for 200 ms

  }
  FuncScore(); // calls the robot score function 

}

void FuncScore() // function that allows the robot to score 
{
  currTime = millis(); // sets currTime to millis 
  while (millis() - currTime < 3000) // for 3 seconds 
  {
    RightMotor -> setSpeed(0); // stop the robot 
    LeftMotor -> setSpeed(0);
    PaintRoller.write(40); // shoot the ball out 

  }

  FuncTurnRestart(); // calls the function turn restar t


}
void FuncTurnRestart() // function so that after the robot scores it can turn and restart 
{
  currTime = millis(); // set currTime to millis 
  while (millis() - currTime < 2200) // for 2.2s 
  {
    FuncLimitSwitch(); // checks limit switches
    RightMotor -> setSpeed(RightMinMotorSpeed); // sets the speed of both motors to min speed 
    LeftMotor -> setSpeed(LeftMinMotorSpeed); 
    RightMotor -> run(BACKWARD); // turns the robot right 
    LeftMotor -> run(FORWARD);
  }
  roboState = Find_Ball; // set robot state to find ball 
}
void FuncLimitSwitch() // function that checks the status of the limit switches 
{

  currTimeLimit = millis(); // sets currTimeLimt equal to millis 
  funcIR_Dist_Sense(); // checks location 
  if (digitalRead(SWITCHR) == HIGH) // if the right limit switch is clicked  
  {
    Serial.println("Right press"); // print right press for debugging 
    if (cmRight < cmLeft) // if the left has more room 
    {
      while (millis() - currTimeLimit < 1500) // for 1.5 seconds 
      {
        RightMotor -> setSpeed(RightMinMotorSpeed); // sets both motors to min speed
         LeftMotor -> setSpeed(LeftMinMotorSpeed);
        RightMotor -> run(BACKWARD); // runs both motors backward 
        LeftMotor -> run(BACKWARD); 
      }

      currTimeLimit = millis(); // set currTimeLimite equal to millis 
      while (millis() - currTimeLimit < 1700) // for 1700 ms 
      {
        RightMotor -> setSpeed(RightMinMotorSpeed); // sets both motors to min speed
         LeftMotor -> setSpeed(LeftMinMotorSpeed);
        RightMotor -> run(FORWARD); // turns left
        LeftMotor -> run(BACKWARD);
      }
    }
  }
  currTimeLimit = millis(); // set currTimeLimit equal to millis 
  funcIR_Dist_Sense();// checks location 
  if (digitalRead(SWITCHL) == HIGH) // check to see if  left limit switch has been pressed 
  {
    if (cmLeft <= cmRight) // if right has more room 
    {
      Serial.println("Left Press");
      while (millis() - currTimeLimit < 1500) // for 1.5 s
      {
         RightMotor -> setSpeed(RightMinMotorSpeed); // sets both motors to min speed
         LeftMotor -> setSpeed(LeftMinMotorSpeed);
        RightMotor -> run(BACKWARD); // run both motors backward
        LeftMotor -> run(BACKWARD);
       
      }
      currTimeLimit = millis(); // sets currTimeLimit equal to millis 
      while (millis() - currTimeLimit < 1700) // for 1700 ms
      {
         RightMotor -> setSpeed(RightMinMotorSpeed); // sets both motors to min speed
         LeftMotor -> setSpeed(LeftMinMotorSpeed);
        RightMotor -> run(BACKWARD); // turn right 
        LeftMotor -> run(FORWARD);
      }
    }
  }
   RightMotor -> setSpeed(rightTriangleFunc); // sets both motors to min speed
    LeftMotor -> setSpeed(leftTriangleFunc);
}
void FuncLimitSwitchWall() // function to check limit switches while it follows a wall
{
  currTimeLimit1 = millis(); // sets currTimeLimit1 equal to millis 
  if (digitalRead(SWITCHR) == HIGH) // 
  {
    Serial.println("Right press"); // print right press for debugging
    while (millis() - currTimeLimit1 < 500) // for 500 ms 
    {
      RightMotor -> run(BACKWARD); // run both motors backward
      LeftMotor -> run(BACKWARD);
      //PaintRoller.write(100);
    }
    currTimeLimit1 = millis(); //sets currTimeLimit1 equal to millis 
    while (millis() - currTimeLimit1 < 300)
    {
      RightMotor -> run(FORWARD); // turn left  
      LeftMotor -> run(BACKWARD); 
    }

  }
  currTimeLimit1 = millis(); // set currTimeLimit1 equal to millis 
  if (digitalRead(SWITCHL) == HIGH) 
  {
    Serial.println("Left Press"); // print left press for debugging 
    while (millis() - currTimeLimit1 < 500) // for 500 ms 
    {
      RightMotor -> run(BACKWARD); // runs both motors backwards
      LeftMotor -> run(BACKWARD);
    }
    currTimeLimit1 = millis(); //sets currTimeLimit1 equal to millis 
    while (millis() - currTimeLimit1 < 300) // for 300 ms
    {
      RightMotor -> run(BACKWARD); // turns right 
      LeftMotor -> run(FORWARD);
    }
  }
}
