// Copyright (C) 2006 Klaas Gadeyne <first dot last at gmail dot com>
//                    Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
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
