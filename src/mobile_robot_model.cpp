// Jarvis Schultz
// November 10, 2011
//
// Library for simulating the noise in a mobile robot as part of the
// EKF implementation.  Adapted from an example on the orocos/ bfl
// website
// people.mech.kuleuven.be/~tdelaet/bfl_doc/getting_started_guide/
// http://www.orocos.org/bfl
//

#include "../include/mobile_robot.h"

using namespace MatrixWrapper;

namespace BFL
{
    MobileRobot::MobileRobot(MatrixWrapper::ColumnVector _init):
        _state(STATE_SIZE)
    {
	// set initial state
	for (unsigned int i=1; i<=STATE_SIZE; i++)
	    _state(i) = _init(i);
	
	// sys noise
	ColumnVector sys_noise_Mu(STATE_SIZE);
	sys_noise_Mu(1) = MU_SYSTEM_NOISE_X_ROB;
	sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y_ROB;
	sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA_ROB;
	SymmetricMatrix sys_noise_Cov(STATE_SIZE);
	sys_noise_Cov = 0.0;
	sys_noise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X_ROB;
	sys_noise_Cov(1,2) = 0.0;
	sys_noise_Cov(1,3) = 0.0;
	sys_noise_Cov(2,1) = 0.0;
	sys_noise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y_ROB;
	sys_noise_Cov(2,3) = 0.0;
	sys_noise_Cov(3,1) = 0.0;
	sys_noise_Cov(3,2) = 0.0;
	sys_noise_Cov(3,3) = SIGMA_SYSTEM_NOISE_THETA_ROB;
	_system_Uncertainty = new Gaussian(sys_noise_Mu, sys_noise_Cov);

	// create the model
	_sys_pdf = new NonLinearAnalyticConditionalGaussianMobile(*_system_Uncertainty);
	_sys_model = new AnalyticSystemModelGaussianUncertainty(_sys_pdf);

    }

    MobileRobot::~MobileRobot()
    {
	delete _system_Uncertainty;
	delete _sys_pdf;
	delete _sys_model;
	delete _measurement_Uncertainty;
	delete _meas_pdf;
	delete _meas_model;
    }

    void
    MobileRobot::Move(ColumnVector inputs)
    {
	_state = _sys_model->Simulate(_state,inputs);
    }

    ColumnVector
    MobileRobot::GetState()
    {
	return _state;
    }
}

