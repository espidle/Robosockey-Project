#include <RobotButton.h>

/* Author: Eric Spidle
 * Date: 10-31-19
 * Class: EGR 107_03 Team 6
 * Description: This code was made to test the drive forward library. The drive forward library 
 * drives two DC motors forward. This library only works with a Adafruit Motorshieldv2 and only DC Motors. 
 * The user has the ability to select the speed and the pins to which the motors are connected
 * Misc: Please note the adafruit library is already included within this library. When a class instance 
 * is created the sheild is intialized so there is no need to begin the motorsheild it is already done
 */
#include <DriveForward.h>// used to include the library into the program

DriveForward DriveForward; // This creates an instance of the class and begins the motorshield 
// Please note each function requires this instace of the class to be made this NEEDS to be included above your setup code
// Please also note that besides when creating instance of the class the function must begin with DriveForward.funcName or they will not compile 
//RobotButton RobotButton(13);

void setup() {
  //RobotButton.ButtonPress();
  Serial.begin(9600); // initaties Serial monitor for debugging and testing purposes if needed
  
  DriveForward.AssignMotorPins(1,4); // Using this function allows the user to set the desired DC motor pins each as a parameter of the function
  
  DriveForward.DetermineSpeed(45,35); // Using this function determines the speed of each motor. 
  //Please note that the first number corrolates with the first motor pin set in the AssignMotorPins 
  //func and the second number correlates to the second motor pin set in AssignMotorPins
}

void loop() {
  
 DriveForward.GoForward(); // This runs the DC Motors forward at the desired speed set above on the desired pins also set above
}
