/* Author: Eric Spidle
 * Date: 10-31-19
 * Class: EGR 107_03 Team 6
 * Description: This is the header file for the DriveForward Library. It contains the class 
 * and all the variables and functions within the class needed for the library to function 
 */
#ifndef DriveForward_h // checks to see if DriveForward.h has already been defined
#define DriveForward_h // If not it defines DriveForward.h as everything below
#include "Arduino.h" // Include regular Arduino functions. This is needed because it is a .h and not a .ino
#include <Adafruit_MotorShield.h> // Include the Adafruit_Motorshieldv2 library so that this library can be used with our robot's motorshield
#include <Wire.h> // Part of the Motorshield library it is needed to run the motors 

class DriveForward // Creates a class with the name DriveForward
{ 
  public: // variables and functions that are accessible to everyone who includes my library in their code
  
  DriveForward(); // Constructor of the class this creates an instance of the class and is needed in every code that uses the class
  
  void AssignMotorPins(int port1, int port2); // function of the class that allows users to define motorpins for their DC Motors
  
  void DetermineSpeed(int desiredSpeedOne, int desiredSpeedTwo); // function of the class that allows the user to set the speed for each motor
  
  void GoForward(); // function of the class that drives the DC motors forward
  
  
  private: // variables and functions that are not acssesible to everyone and can only  be changed through the class 
  
  Adafruit_MotorShield _AFMS; // private version of AdafruitMotorshield constructor
  int _port1; // private version of port1 that stores the first motor pin
  int _port2; // private version of port2 that stores the second motor pin
  Adafruit_DCMotor *_myMotor1;  // private version of myMotor1 that allows the DC motors to be given commands 
  Adafruit_DCMotor *_myMotor2; // private version of myMotor2 that allows the DC motors to be given commands
  
}; // required syntax to end a class 

#endif // ends the definition of DriveForward.h
