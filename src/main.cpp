/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ryanspromac                                               */
/*    Created:      9/26/2023, 2:14:30 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;

const double wheelSize = 4.125;

// define your global instances of motors and other devices here
brain Brain;

controller Controller = controller(primary);
motor LF = motor(PORT20, ratio6_1, false);
motor LM = motor(PORT9, ratio6_1, false);
motor LB = motor(PORT10, ratio6_1, false);
motor RF = motor(PORT12, ratio6_1, true);
motor RM = motor(PORT2, ratio6_1, true);
motor RB = motor(PORT1, ratio6_1, true);
motor arm = motor(PORT6, ratio36_1, false);  
motor intake = motor(PORT8, ratio6_1, false);
motor puncher = motor(PORT4, ratio18_1, true); 
inertial IMU = inertial(PORT18); 
rotation odom = rotation(PORT19); //wrong port


/*---------------------------------------------------------------------------*/
/*                          Calculate Displacement                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/

 #define inchesToDegrees(inches) 360*inches/(wheelSize*M_PI);
 #define degreesToInches(degrees)  degrees*wheelSize*M_PI/360;

double currentX =  0;
double currentY = 0;
double prevX = 0;
double prevY = 0;

double robotPosition[3]; //0=x 1=y 2=rotation

int calculatePosition(){
  while(true){
    currentX = degreesToInches(RF.position(degrees));
    currentY = degreesToInches(odom.rotation(degrees));

    robotPosition[0] = (currentX - prevX) * cos(IMU.rotation(degrees)); //x
    robotPosition[1] = (currentY - prevY) * sin(IMU.rotation(degrees)); //y
    robotPosition[2] = IMU.rotation(degrees);

    prevX = currentX;
    prevY = currentY;

    task::sleep(5);
  }
  return 0;

}

/*---------------------------------------------------------------------------*/
/*                          PID Function                                     */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool PID_enable = true;
bool PID_reset = true;

double KP=0.2,KI=0.0,KD=0.000;
double originalKP = KP;
double error=0,prevError=0,totalError=0;
int desiredPosition=0;

double inertial_KP = 0.3,inertial_KI=0.0,inertial_KD=0.00;
double inertial_error=0,inertial_prevError=0,inertial_totalError=0;
int desiredDirection=0;


int PID_running(){
  LF.spin(forward);
  LM.spin(forward);
  LB.spin(forward);
  RF.spin(forward);
  RM.spin(forward);
  RB.spin(forward);

  while(PID_enable){
    if(PID_reset){
      PID_reset = false;
      LF.setPosition(0, degrees);
      LM.setPosition(0, degrees);
      LB.setPosition(0, degrees);
      RF.setPosition(0, degrees);
      RM.setPosition(0, degrees);
      RB.setPosition(0, degrees);
    }
    int currentPosition;
    currentPosition = (LF.position(degrees)+LM.position(degrees)+LB.position(degrees)+RF.position(degrees)+RM.position(degrees)+RB.position(degrees))/6;
    error = desiredPosition - currentPosition;
    totalError += error;
    int movePower = KP * error + KI * (error - prevError) + KD * totalError;

    inertial_error = desiredDirection - IMU.rotation(degrees);
    inertial_totalError += inertial_error;
    int turnPower = inertial_KP *  inertial_error + inertial_KI * (inertial_error - inertial_prevError) + inertial_KD * inertial_totalError;
    LF.setVelocity(movePower + turnPower, percent);
    LM.setVelocity(movePower + turnPower, percent);
    LB.setVelocity(movePower + turnPower, percent);
    RF.setVelocity(movePower - turnPower, percent);
    RM.setVelocity(movePower - turnPower, percent);
    RB.setVelocity(movePower - turnPower, percent);

    prevError=error;
    inertial_prevError=inertial_error;

    task::sleep(20);
  }
  return 0;
}

void brainDisplay(){
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("x: %f, y: %f, angle: %f", robotPosition[0], robotPosition[1], robotPosition[2]);
    Brain.Screen.newLine();
    Brain.Screen.print("RF: %f RM: %f RB: %f LF: %f LM: %f LB: %f", RF.position(degrees), RM.position(degrees), RB.position(degrees), LF.position(degrees), LM.position(degrees), LB.position(degrees), );
    Brain.Screen.newLine();

    wait(100, msec);
}
void resetPID(){
  PID_reset = true;
  desiredPosition = 0;
  desiredDirection = IMU.rotation(degrees);
}

void moveTo(double d,int t){
  resetPID();
  desiredPosition = inchesToDegrees(d);
  task::sleep(t);
  resetPID();
}

void turnTo(int d, int t){
  resetPID();
  desiredDirection = d;
  task::sleep(t);
  resetPID();
}

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
  PID_enable = true;

  arm.setMaxTorque(100, percent);
  arm.setStopping(hold);
  intake.setMaxTorque(100, percent);
  intake.setStopping(brake);
  LF.setStopping(brake);
  LM.setStopping(brake);
  LB.setStopping(brake);
  RF.setStopping(brake);
  RM.setStopping(brake);
  RB.setStopping(brake);
  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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
  task PID_task = task(PID_running);
  moveTo(20, 1000);
  turnTo(90, 1000);
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
  task display = task(brainDisplay);
  while (1) {
    int axis3 = Controller.Axis3.position();
    int axis1 = Controller.Axis1.position();

    LF.spin(forward, 0.12*(-axis3-axis1), volt);
    LM.spin(forward, 0.12*(-axis3-axis1), volt);
    LB.spin(forward, 0.12*(-axis3-axis1), volt);
    RF.spin(forward, 0.12*(-axis3+axis1), volt);
    RM.spin(forward, 0.12*(-axis3+axis1), volt);
    RB.spin(forward, 0.12*(-axis3+axis1), volt);

    if(Controller.ButtonL1.pressing()){
      arm.spin(forward, 100, percent);
    } else if(Controller.ButtonL2.pressing()){
      arm.spin(reverse, 100, percent);
    } else{
      arm.stop();
    }

    if(Controller.ButtonR1.pressing()){
      intake.spin(forward, 100, percent);
    } else if(Controller.ButtonR2.pressing()){
      intake.spin(reverse, 100, percent);
    } else{
      intake.stop();
    }

    if(Controller.ButtonUp.pressing()){
      puncher.spin(forward, 100, percent);
    } else if(Controller.ButtonDown.pressing()){
      puncher.spin(reverse, 100, percent);
    } else{
      puncher.stop();
    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
