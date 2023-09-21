#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor WheelLF = motor(PORT9, ratio6_1, true);
motor WheelLM = motor(PORT20, ratio6_1, true);
motor WheelLB = motor(PORT10, ratio6_1, true);
motor WheelRB = motor(PORT2, ratio6_1, false);
motor WheelRM = motor(PORT1, ratio6_1, false);
motor WheelRF = motor(PORT12, ratio6_1, false);
inertial IMU = inertial(PORT13);
motor Intake = motor(PORT14, ratio6_1, false);
motor Flywheel = motor(PORT15, ratio6_1, true);
digital_out expansion1 = digital_out(Brain.ThreeWirePort.A);
digital_out AngleAdjuster = digital_out(Brain.ThreeWirePort.C);
digital_out Blockers = digital_out(Brain.ThreeWirePort.B);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}