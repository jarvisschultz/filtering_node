// Jarvis Schultz
// November 10, 2011

// prototypes for simulating a mobile robot adapted from an example at
// the orocos/ bfl website
// http://people.mech.kuleuven.be/~tdelaet/bfl_doc/getting_started_guide/

#ifndef MOBILE_ROBOT_HPP
#define MOBILE_ROBOT_HPP


#include <model/analyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/gaussian.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/matrix/vector_wrapper.h>

#include "mobile_robot_constants.h"
#include "filter_constants.h"


namespace BFL{

    // Class for simulating a mobile robot.
    class MobileRobot
    {
    public:
	// Constructor
	MobileRobot(MatrixWrapper::ColumnVector _init);
	// Destructor
	~MobileRobot();

	// Method for integrating model:
	void Move(MatrixWrapper::ColumnVector inputs);
	// Method for returning the state:
	MatrixWrapper::ColumnVector GetState(); 

    private:
	Gaussian* _system_Uncertainty;
	NonLinearAnalyticConditionalGaussianMobile* _sys_pdf;
	AnalyticSystemModelGaussianUncertainty* _sys_model;
	Gaussian* _measurement_Uncertainty;
	LinearAnalyticConditionalGaussian* _meas_pdf;
	LinearAnalyticMeasurementModelGaussianUncertainty* _meas_model;
	MatrixWrapper::ColumnVector _state;
    };
}

#endif
