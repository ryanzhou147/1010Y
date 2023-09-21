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
// WheelLF              motor         9               
// WheelLM              motor         20              
// WheelLB              motor         10              
// WheelRB              motor         2               
// WheelRM              motor         1               
// WheelRF              motor         12              
// IMU                  inertial      13              
// Intake               motor         14              
// Flywheel             motor         15              
// expansion1           digital_out   A               
// AngleAdjuster        digital_out   C               
// Blockers             digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"
#include <iostream> 
#include <string>
#include <string.h>
#include <cmath>

using namespace vex;


competition Competition;


//use to calculate the wheel size
const double pi = 3.14159;
const double wheelSize = 3.25;

int inchesToDegrees(double inches){
  return (360*inches)/(wheelSize*pi);
}

//PID variable
bool PID_enable = true;
bool PID_reset = true;
//0.11, 0.0002
double KP=0.14,KD=0.00002,KI=0.000;
const double originalKP = KP;
const double originalKD = KD;
double error=0,prevError=0,totalError=0;
int desiredPosition=0;

double intertial_KP = 0.53, intertial_KD=0.000085,intertial_KI=0.000;
double intertial_error=0,intertial_prevError=0,intertial_totalError=0;
int desiredDirection=0;

int maxPIDPercent = 80;

int PID_running(){
  WheelLF.spin(forward);
  WheelLM.spin(forward);
  WheelLB.spin(forward);
  WheelRF.spin(forward);
  WheelRM.spin(forward);
  WheelRB.spin(forward);

  while(PID_enable){
    if(PID_reset){
      PID_reset = false;
      WheelLF.setPosition(0,degrees);
      WheelLM.setPosition(0, degrees);
      WheelLB.setPosition(0,degrees);
      WheelRF.setPosition(0,degrees);
      WheelRM.setPosition(0, degrees);
      WheelRB.setPosition(0,degrees);
    }
    int currentPosition;
    currentPosition = (WheelLF.position(degrees)+WheelLM.position(degrees)+WheelLB.position(degrees)+WheelRF.position(degrees)+WheelRM.position(degrees)+WheelRB.position(degrees))/6;
    error = desiredPosition - currentPosition;
    totalError += error;
    int movePower = KP * error + KI * (error - prevError) + KD * totalError;

    intertial_error = desiredDirection - IMU.rotation(degrees);
    intertial_totalError += intertial_error;
    int turnPower = intertial_KP *  intertial_error + intertial_KI * (intertial_error - intertial_prevError) + intertial_KD * intertial_totalError;
    WheelLF.setVelocity(std::min(movePower + turnPower, maxPIDPercent), percent);
    WheelLM.setVelocity(std::min(movePower + turnPower, maxPIDPercent), percent);
    WheelLB.setVelocity(std::min(movePower + turnPower, maxPIDPercent), percent);
    WheelRF.setVelocity(std::min(movePower - turnPower, maxPIDPercent), percent);
    WheelRM.setVelocity(std::min(movePower - turnPower, maxPIDPercent), percent);
    WheelRB.setVelocity(std::min(movePower - turnPower, maxPIDPercent), percent);

    prevError=error;
    intertial_prevError=intertial_error;

    task::sleep(20);
  }
  return 0;
}

void resetPID(){
  PID_reset = true;
  desiredPosition = 0;
  desiredDirection = IMU.rotation(degrees);
}

void move(double d,int t){
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

bool flywheelOn=false; 
bool arcadeDrive = true;
bool angleAdjuster = false;
bool lockWheels = false;

void lockWheelsOn() {
  lockWheels = !lockWheels;
}
void flywheelSwitch(){
  flywheelOn = !flywheelOn;
}
void switchDrive(){
  arcadeDrive = !arcadeDrive;
}
void angleAdjusterSwitch(){
  angleAdjuster = !angleAdjuster;
}
  
  double flywheelVoltage = 8.8;

void increaseFlywheelVoltage(){
  flywheelVoltage = flywheelVoltage + 0.2;
}
void decreaseFlywheelVoltage(){
  flywheelVoltage = flywheelVoltage - 0.2;
}

  double maxRPM = 0;


int test(){
  while(1){
    // Brain.Screen.print(WheelLF.position(degrees));
    // Brain.Screen.print(" ");
    // Brain.Screen.print(WheelLM.position(degrees));
    // Brain.Screen.print(" ");
    // Brain.Screen.print(WheelLB.position(degrees));
    // Brain.Screen.print(" ");
    // Brain.Screen.print(WheelRF.position(degrees));
    // Brain.Screen.print(" ");
    // Brain.Screen.print(WheelRM.position(degrees));
    // Brain.Screen.print(" ");
    // Brain.Screen.print(WheelRB.position(degrees));
    // Brain.Screen.print(" ");
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Flywheel rpm: %f", Flywheel.velocity(rpm));
    Brain.Screen.newLine();
    if(Flywheel.velocity(rpm)>405){
      Brain.Screen.clearScreen();
      Brain.Screen.print(Brain.Timer.value());
      break;
    }
    Brain.Screen.print("Intake rpm: %f", Intake.velocity(rpm));
    Brain.Screen.newLine();
    Brain.Screen.print("Drivetrain average rpm: %f", 
                      (WheelLF.velocity(rpm) + WheelLM.velocity(rpm) + WheelLB.velocity(rpm) + 
                      WheelRF.velocity(rpm) + WheelRM.velocity(rpm) + WheelRB.velocity(rpm))/6);
    Brain.Screen.newLine();
    Brain.Screen.print(IMU.heading(degrees));
    Brain.Screen.newLine();
    Brain.Screen.print("Max RPM: %f", maxRPM);
    Brain.Screen.newLine();
    Brain.Screen.print(" Flywheel voltage: %f", flywheelVoltage);

    task::sleep(100);
    
  }
  return 0;
}
bool intakeState = false;
bool indexerState = false;
bool slowMode = false;
bool die = false;

int robotDaemon() {
  while(true){
     if (intakeState) {
        Intake.spin(forward, 10, volt);
     } else if (indexerState) {
        Intake.spin(reverse);
     } else {
        Intake.stop();
     }
      if (slowMode){
        KP = originalKP/10;
        KD = 0;
      } else {
        KP=originalKP;
        KD=originalKD;
      }
      if(die){
          break;
      }
      task::sleep(20);
  }
  return 0;
}

void preAuton(){
  expansion1.set(false);
  Blockers.set(false);
  AngleAdjuster.set(false);

  WheelLF.setPosition(0,degrees);
  WheelLM.setPosition(0,degrees);
  WheelLB.setPosition(0,degrees);
  WheelRF.setPosition(0,degrees);
  WheelRM.setPosition(0,degrees);
  WheelRB.setPosition(0,degrees);

  Intake.setMaxTorque(100, percent);
  Intake.setVelocity(100, percent);

  WheelLF.setStopping(brake);
  WheelLM.setStopping(brake);
  WheelLB.setStopping(brake);
  WheelRF.setStopping(brake);
  WheelRM.setStopping(brake);
  WheelRB.setStopping(brake);

  Flywheel.setMaxTorque(100, percent);
  PID_enable = true;
  Flywheel.setStopping(coast);
  
}
void shoot3(){
  indexerState = true;
  vex::task::sleep(150);
  indexerState = false;
  vex::task::sleep(600);
  indexerState = true;
  vex::task::sleep(150);
  indexerState = false;
  vex::task::sleep(600);
  indexerState = true;
  vex::task::sleep(200);
  indexerState = false;
}

int endgameRumble(){
  while(true){
      if(Brain.Timer.value()>=95 && Brain.Timer.value()<=102){
        Controller1.rumble(".");
      }
      if(Brain.Timer.value()>=102 && Brain.Timer.value()<=105){
        Controller1.rumble("-");
      }
      task::sleep(50);
  }
  return 0;
}

void Autonomous(){
  preAuton();
  task PID_task = task(PID_running);
  task run_Daemon = task(robotDaemon);
  task testing = task(test);

  Brain.Timer.reset();
  expansion1.set(false);
  Blockers.set(false);
  AngleAdjuster.set(false);

  move(24, 2000);
  turnTo(90, 1000);
  //Auto in front with voltage
  if(0){
      
      Flywheel.spin(forward, 12, volt);
      move(2, 500);
      intakeState = true;
      vex::task::sleep(300);
      intakeState = false;
      move(-4, 600);

      vex::task::sleep(1000);
      turnTo(-14, 1000);
      task::sleep(2000);
      Flywheel.spin(forward, 10.6, volt);
      indexerState = true;
      vex::task::sleep(400);
      indexerState = false;
      Flywheel.spin(forward, 9, volt);

      vex::task::sleep(500);
      turnTo(-133, 1000);
      intakeState = true;
      move(30, 700);
      slowMode = true;
      move(70, 700);
      Flywheel.spin(forward, 8, volt);
      slowMode = false;
      move(15, 800);
      Flywheel.spin(forward, 9.3, volt);

      turnTo(-32, 1000);
      intakeState = false;
      vex::task::sleep(500);
      indexerState = true;
      task::sleep(200);
      indexerState = false;
      task::sleep(400);
      indexerState = true;
      task::sleep(200);
      indexerState = false;
      task::sleep(400);
      indexerState = true;
      task::sleep(200);
      indexerState = false;

      die = true;
  }

  //Shoot first in front!!!
  if(false){
    Flywheel.spin(forward, 12, volt);
    vex::task::sleep(4250);
    indexerState = true;
    vex::task::sleep(200);
    indexerState = false;
    vex::task::sleep(800);
    indexerState = true;
    vex::task::sleep(200);
    indexerState = false;
    turnTo(8, 700);
    Flywheel.spin(forward, 5, volt);
    move(2, 500);

    intakeState = true;
    vex::task::sleep(220);
    intakeState = false;

    move(-5, 600);
    turnTo(-125, 1000);
    intakeState = true;
    move(35, 600);
    maxPIDPercent = 40;
    move(23, 600);
    maxPIDPercent = 80;
    Flywheel.spin(forward, 9, volt);
    vex::task::sleep(1500);
    turnTo(-23, 700);
    intakeState = false;
    Flywheel.spin(forward, 9.6, volt);
    vex::task::sleep(1000);

    indexerState = true;
    vex::task::sleep(200);
    indexerState = false;
    Flywheel.spin(forward, 11, volt);
    vex::task::sleep(700);
    indexerState = true;
    vex::task::sleep(200);
    indexerState = false;
    vex::task::sleep(700);
    indexerState = true;
    vex::task::sleep(200);
    indexerState = false;

    die = true;
  }

  //Shoot first not in front
  if(false){
    intakeState = true;
    Flywheel.spin(forward, 9, volt);
    maxPIDPercent = 40;
    move(65, 1000);
    vex::task::sleep(1500);
    maxPIDPercent = 80;
    turnTo(183, 1500);

    vex::task::sleep(1000);
    intakeState = false;
    vex::task::sleep(500);

    indexerState = true;
    vex::task::sleep(150);
    indexerState = false;
    Flywheel.spin(forward, 10, volt);
    vex::task::sleep(800);
    indexerState = true;
    vex::task::sleep(150);
    indexerState = false;
    vex::task::sleep(2000);
    indexerState = true;
    vex::task::sleep(150);
    indexerState = false;

    Flywheel.stop(coast);
    turnTo(105, 1000);
    move(47, 1000);
    turnTo(155, 1000);
    move(6, 1000);
    indexerState = true;
    vex::task::sleep(200);
    indexerState = false;
    move(-5, 1000);

    die=true;

  }

  //Auto not in front with voltage !!!
  if(0){

    Flywheel.spin(forward, 11.5, volt);
    move(30, 1000);
    turnTo(90, 1000);
    move(7, 1000);
    intakeState = true;
    vex::task::sleep(300);
    intakeState = false;

    move(-8, 500);
    turnTo(101, 1500);
    vex::task::sleep(800);
    Flywheel.spin(forward, 9.8, volt);
    indexerState = true;
    vex::task::sleep(150);
    indexerState = false;
    vex::task::sleep(400);
    indexerState = true;
    vex::task::sleep(150);
    indexerState = false;

    turnTo(222, 1000);
    intakeState = true;
    move(95, 2000);
    // move(62, 1000);
    // turnTo(225, 700);
    // vex::task::sleep(500);
    // move(60, 500);
    Flywheel.spin(forward, 8.2, volt);
    vex::task::sleep(1000);

    turnTo(136, 1000);
    move(-10, 1000);
    intakeState = false;
    Flywheel.spin(forward, 8.3, volt);
    vex::task::sleep(200);
    indexerState = true;
    vex::task::sleep(200);
    indexerState = false;
    Flywheel.spin(forward, 10, volt);
    vex::task::sleep(300);
    indexerState = true;
    vex::task::sleep(200);
    indexerState = false;
    Flywheel.spin(forward, 11, volt);
    vex::task::sleep(450);
    indexerState = true;
    vex::task::sleep(200);
    indexerState = false;
    die=true;
  }

  //Auto Messing up opponents
 

}
void drivercontrolInit(){
  PID_enable = false;
  die=true;
  WheelLF.setStopping(brake);
  WheelLM.setStopping(brake);
  WheelLB.setStopping(brake);
  WheelRF.setStopping(brake);
  WheelRM.setStopping(brake);
  WheelRB.setStopping(brake);

  Intake.setMaxTorque(100, percent);
  Intake.setVelocity(100, percent);

  Flywheel.setMaxTorque(100, percent);
  Flywheel.setStopping(coast);

  expansion1.set(false);
  AngleAdjuster.set(false);
  Blockers.set(false);

}
void Drivercontrol(){
  drivercontrolInit();
  Brain.Timer.reset();
  int wheelSpeed = 100;
  task endgame = task(endgameRumble);
  Controller1.ButtonL1.pressed(flywheelSwitch);
  Controller1.ButtonX.pressed(increaseFlywheelVoltage);
  Controller1.ButtonY.pressed(lockWheelsOn);
  Controller1.ButtonDown.pressed(decreaseFlywheelVoltage);
  task testing = task(test);

  while(true){
    int axis3 = Controller1.Axis3.position() * wheelSpeed / 100;
    int axis1 = Controller1.Axis1.position() * wheelSpeed / 100;
    int axis2 = Controller1.Axis2.position() * wheelSpeed / 100;

    if(arcadeDrive && !lockWheels){
      WheelLF.spin(forward, 0.12*(axis3+axis1), volt);
      WheelLM.spin(forward, 0.12*(axis3+axis1), volt);
      WheelLB.spin(forward, 0.12*(axis3+axis1), volt);
      WheelRF.spin(forward, 0.12*(axis3-axis1), volt);
      WheelRM.spin(forward, 0.12*(axis3-axis1), volt);
      WheelRB.spin(forward, 0.12*(axis3-axis1), volt);
    } else if (!arcadeDrive && !lockWheels) {
      WheelLF.spin(forward, 0.12*(axis3), volt);
      WheelLM.spin(forward, 0.12*(axis3), volt);
      WheelLB.spin(forward, 0.12*(axis3), volt);
      WheelRF.spin(forward, 0.12*(axis2), volt);
      WheelRM.spin(forward, 0.12*(axis2), volt);
      WheelRB.spin(forward, 0.12*(axis2), volt);
    } else {
      WheelLF.stop(hold);
      WheelLM.stop(hold);
      WheelLB.stop(hold);
      WheelRF.stop(hold);
      WheelRM.stop(hold);
      WheelRB.stop(hold);
    }

    if(Controller1.ButtonR1.pressing()){
      Intake.spin(forward, 10, volt);
    } else if(Controller1.ButtonUp.pressing() || Controller1.ButtonR2.pressing()){
      Intake.spin(reverse, 12, volt);
    } else{
      Intake.stop(hold);
    }
    if(flywheelOn){
      Flywheel.spin(forward,flywheelVoltage, volt);
    } else{
      Flywheel.spin(forward, 0, volt);
    }

    if(Controller1.ButtonB.pressing() && Brain.Timer.value() >= 95) {
        expansion1.set(true);
    }

      if(Controller1.ButtonRight.pressing()){
        AngleAdjuster.set(false);
        flywheelVoltage = 9;
      } else if(Controller1.ButtonLeft.pressing()){
        AngleAdjuster.set(true);
        flywheelVoltage = 8.5;
      }

      if(Controller1.ButtonA.pressing() && Brain.Timer.value()>=95){
        Blockers.set(true);
      }
        task::sleep(20);
    }
}
//12 volts 480 rpm
//10 volts 405 rpm
//6.77s to get 405 rpm at 12 volt
int main() {
  vexcodeInit();
  Competition.autonomous(Autonomous);
  Competition.drivercontrol(Drivercontrol);
}