/* Author: Eric Spidle
 * Date: 10-31-19
 * Class: EGR 107_03 Team 6
 * Description: This is the .cpp file for the DriveForward.h library. It contains all the functions and what each function 
 * does within it. This where all the code that does the actual work goes.
 */
#include "Arduino.h" // // Include regular Arduino functions. This is needed because it is a .h and not a .ino
#include <Adafruit_MotorShield.h> // // Include the Adafruit_Motorshieldv2 library so that this library can be used with our robot's motorshield
#include <Wire.h> // Part of the Motorshield library it is needed to run the motors 
#include "DriveForward.h" // Includes the header file in the .cpp file so the compilier knows to read code from the class

DriveForward:: DriveForward() // constructor of the class when called creates an instance of the class this 
{
   _AFMS = Adafruit_MotorShield(); // starts the adafruit_motorshieldv2 so it can be used
  
}
void DriveForward::AssignMotorPins(int port1, int port2) // function for assigning motor pins it takes in two pins as the parameters 
{
  _port1 = port1; // sets the private pin variable equal to the public pin variable for pin1
  _port2 = port2; // sets the private pin variable equal to the public pin variable for pin2
  
  Adafruit_DCMotor *myMotor1 = _AFMS.getMotor(_port1); // intializes a DC motor variable for the DC motors and sets the pin equal to the functions parameter
  Adafruit_DCMotor *myMotor2 = _AFMS.getMotor(_port2); // intializes a DC motor variable for the DC motors and sets the pin equal to the functions parameter
  
  *_myMotor1 = *myMotor1; // sets private motor variable for the DC motor to the same one declared above so that it can be used later in the code
  *_myMotor2 = *myMotor2; // sets private motor variable for the DC motor to the same one declared above so that it can be used later in the code

 }
void DriveForward::DetermineSpeed(int desiredSpeedOne, int desiredSpeedTwo)  // function that allows the user to set the motorspeed for each motor
{

  _myMotor1 -> setSpeed(desiredSpeedOne); // sets the motor to the desired speed input by the user in the functions parameter for motor 1
  _myMotor2 -> setSpeed(desiredSpeedTwo); // sets the motor to the desired speed input by the user in the functions parameter for motor 2
}
void DriveForward::GoForward() // function that drives the DC motors forward
{
  _myMotor1 -> run(FORWARD); // takes the the first motor and runs it forward
  _myMotor2 -> run(FORWARD); // takes the second motor and runs it forward
}
