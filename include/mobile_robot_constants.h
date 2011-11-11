// Jarvis Schultz
// November 10, 2011
//
// Defining constants for the mobile robot simulation.
// 
//

#ifndef __MOBILE_ROBOT_CTS_
#define __MOBILE_ROBOT_CTS_

#include <cmath>

// Sizes
#define STATE_SIZE 3 //state: x,y,theta
#define INPUT_SIZE 2 //input: v*deltat, omega*deltat

// System Noise
#define MU_SYSTEM_NOISE_X 0.0 
#define MU_SYSTEM_NOISE_Y 0.0 
#define MU_SYSTEM_NOISE_THETA 0.0
#define SIGMA_SYSTEM_NOISE_X pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_Y pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_THETA pow(2*M_PI/180,2)

// System Noise for mobile robot simulator
#define MU_SYSTEM_NOISE_X_ROB 0.0 
#define MU_SYSTEM_NOISE_Y_ROB 0.0 
#define MU_SYSTEM_NOISE_THETA_ROB 0.0
#define SIGMA_SYSTEM_NOISE_X_ROB pow(0.001,2)
#define SIGMA_SYSTEM_NOISE_Y_ROB pow(0.001,2)
#define SIGMA_SYSTEM_NOISE_THETA_ROB pow(0.1*M_PI/180,2)

#endif //__MOBILE_ROBOT_CTS
