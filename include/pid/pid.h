// #ifndef PID_H
// #define PID_H
// #include "vex.h" //user-defined header file uses " "
// #include <cmath> //built-in library header file uses < >

// using namespace vex;

// struct PID { //struct = public, class = private, we've got noth' to hide

//    //Settings:
//    double KP;
//    double KI;
//    double KD;

//    double inertial_KP;
//    double inertial_KI;
//    double inertial_KD;

//    //Calculations:
//    double error;  //Target state - Current state = Position
//    double prevError = 0.0;  //Position 20ms ago
//    double totalError = 0.0;  //The total error collected over time

//    double inertial_error;
//    double inertial_prevError = 0.0;
//    double inertial_totalError = 0.0;


// };




// #endif