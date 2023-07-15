/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LDrive               motor_group   2, 4            
// RDrive               motor_group   3, 6            
// lRot                 rotation      11              
// rRot                 rotation      12              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath> //std::abs

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//Settings
double kP = 0.007;
double kI = 0.00;
double kD = 0.01;

double turnkP = 0.005;
double turnkI = 0.00;
double turnkD = 0.01;
int maxTurnIntegral = 300; // These cap the integrals
int maxIntegral = 300;
int integralBound = 3; //If error is outside the bounds, then apply the integral. This is a buffer with +-integralBound degrees

//Autonomous Settings
int desiredValue = 100;
int desiredTurnValue = 30;
float waypoints[][2] = {
      {0.0, 9.0}, // Waypoint 1 (x, y)
      {0.0, 18.0}, // Waypoint 2 (x, y)
      {0.0, 36.0}  // Waypoint 3 (x, y)
  };

int error; //SensorValue - DesiredValue : Position
int prevError = 0; //Position 20 miliseconds ago
int derivative; // error - prevError : Speed
int totalError = 0; //totalError = totalError + error
int currentWaypoint = 0;
int turnError; //SensorValue - DesiredValue : Position
int turnPrevError = 0; //Position 20 miliseconds ago
int turnDerivative; // error - prevError : Speed
int turnTotalError = 0; //totalError = totalError + error

bool resetDriveSensors = false;

//Variables modified for use
bool enableDrivePID = true;

//Pasted from a C++ resource
double signnum_c(double x) {
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return x;
}

int drivePID(){
   while (currentWaypoint < sizeof(waypoints) / sizeof(waypoints[0])) {
    // Get target waypoint coordinates
    desiredValue = waypoints[currentWaypoint][0];
    desiredTurnValue = waypoints[currentWaypoint][1];
   }
   printf("%s\n","pid true");
  while(enableDrivePID){
printf("desired val%d\n",desiredValue);
printf("desired turn val%d\n",desiredTurnValue);
    if (resetDriveSensors) {
      resetDriveSensors = false;
      lRot.setPosition(0,deg);
      rRot.setPosition(0,deg);
    }


    //Get the position of both motors
    int leftRotPosition = lRot.position(degrees)*0.5;
    int rightRotPosition = rRot.position(degrees)*0.5;

    ///////////////////////////////////////////
    //Lateral movement PID
    /////////////////////////////////////////////////////////////////////
    //Get average of the two motors
    int averagePosition = (leftRotPosition + rightRotPosition)/2;

    //Potential
    error = averagePosition - desiredValue;
printf("error value%d\n",error);
    //Derivative
    derivative = error - prevError;

    //Integral
    if(abs(error) < integralBound){
    totalError+=error; 
    }  else {
    totalError = 0; 
    }
    //totalError += error;

    //This would cap the integral
    totalError = abs(totalError) > maxIntegral ? signnum_c(totalError) * maxIntegral : totalError;

    double lateralMotorPower = error * kP + derivative * kD + totalError * kI;
    /////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////
    //Turning movement PID
    /////////////////////////////////////////////////////////////////////
    //Get average of the two motors
    int turnDifference = leftRotPosition - rightRotPosition;

    //Potential
    turnError = turnDifference - desiredTurnValue;

    //Derivative
    turnDerivative = turnError - turnPrevError;

    //Integral
    if(abs(error) < integralBound){
    turnTotalError+=turnError; 
    }  else {
    turnTotalError = 0; 
    }
    //turnTotalError += turnError;

    //This would cap the integral
    turnTotalError = abs(turnTotalError) > maxIntegral ? signnum_c(turnTotalError) * maxIntegral : turnTotalError;

    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;
    /////////////////////////////////////////////////////////////////////
 printf("lateral power%f\n",lateralMotorPower);
  printf("turn power%f\n",turnMotorPower);
    LDrive.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RDrive.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);


    

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }

printf("%s\n","pid false");
  return 1;
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
// printf("%s/n","auton called");

///First set of instructoin
  vex::task billWiTheScienceFi(drivePID);
 resetDriveSensors = true;
  desiredValue = 720;
  desiredTurnValue = 0;

 
   vex::task::sleep(1000);

//secon set of insctrution
 resetDriveSensors = true;
 desiredValue = 360;
  desiredTurnValue = 45;
 
     vex::task::sleep(1000);

     //end pid
enableDrivePID = false;
    resetDriveSensors = true;

LDrive.stop();
RDrive.stop();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

//  enableDrivePID = false;
/*
  ///////////////////////////
  //Settings
  ///////////////////////////////////////////////////////////////////////////

  //Drivetrain
  double turnImportance = 0.5;

  while (1) {

    ///////////////////////////
    //Driver Control
    ///////////////////////////////////////////////////////////////////////////
    double turnVal = Controller1.Axis1.position(percent);
    double forwardVal = Controller1.Axis3.position(percent);

    double turnVolts = turnVal * 0.12;
    double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);

    //0 - 12 = -12
    //0 + 12 = 12(due to cap)


    LeftMotor.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
    RightMotor.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
    ///////////////////////////////////////////////////////////////////////////


    ///////////////////////////
    //Arm Control
    ///////////////////////////////////////////////////////////////////////////
    bool topRightButton = Controller1.ButtonR1.pressing();
    bool bottomRightButton = Controller1.ButtonR2.pressing();

    if (topRightButton){
      ArmMotor.spin(forward, 12.0, voltageUnits::volt);
    }
    else if (bottomRightButton){
      ArmMotor.spin(forward, -12.0, voltageUnits::volt);
    }
    else{
      ArmMotor.spin(forward, 0, voltageUnits::volt);
    }
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////
    //Claw Control
    ///////////////////////////////////////////////////////////////////////////
    bool topLeftButton = Controller1.ButtonL1.pressing();
    bool bottomLeftButton = Controller1.ButtonL2.pressing();

    if (topLeftButton){
      ClawMotor.spin(forward, 12.0, voltageUnits::volt);
    }
    else if (bottomLeftButton){
      ClawMotor.spin(forward, -12.0, voltageUnits::volt);
    }
    else{
      ClawMotor.spin(forward, 0, voltageUnits::volt);
    }
    ///////////////////////////////////////////////////////////////////////////



    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
    */
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  //printf("%s/n","calling auton");
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }

}
