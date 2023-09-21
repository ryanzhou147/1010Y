using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor WheelLF;
extern motor WheelLM;
extern motor WheelLB;
extern motor WheelRB;
extern motor WheelRM;
extern motor WheelRF;
extern inertial IMU;
extern motor Intake;
extern motor Flywheel;
extern digital_out expansion1;
extern digital_out AngleAdjuster;
extern digital_out Blockers;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );