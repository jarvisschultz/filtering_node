// Jarvis Schultz
// November 10, 2011
//
// This file implements the important functions for running a
// nonlinear system model with an EKF.  Adapted from the orocos/ bfl
// website
// http://people.mech.kuleuven.be/~tdelaet/bfl_doc/getting_started_guide/
// http://www.orocos.org/bfl
// 



#include "../include/filter_constants.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng
                                 // libraries
#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
  using namespace MatrixWrapper;


   NonLinearAnalyticConditionalGaussianMobile::NonLinearAnalyticConditionalGaussianMobile(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_MOBILE)
  {
  }


  NonLinearAnalyticConditionalGaussianMobile::~NonLinearAnalyticConditionalGaussianMobile(){}

  ColumnVector NonLinearAnalyticConditionalGaussianMobile::ExpectedValueGet() const
  {
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);
    state(1) += cos(state(3)) * vel(1);
    state(2) += sin(state(3)) * vel(1);
    state(3) += -1.0 * vel(2);
    return state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianMobile::dfGet(unsigned int i) const
  {
    if (i==0)//derivative to the first conditional argument (x)
      {
	ColumnVector state = ConditionalArgumentGet(0);
	ColumnVector vel = ConditionalArgumentGet(1);
	Matrix df(3,3);
	df(1,1)=1;
	df(1,2)=0;
	df(1,3)=-vel(1)*sin(state(3));
	df(2,1)=0;
	df(2,2)=1;
	df(2,3)=vel(1)*cos(state(3));
	df(3,1)=0;
	df(3,2)=0;
	df(3,3)=1;
	return df;
      }
    else
      {
	if (i >= NumConditionalArgumentsGet())
	  {
	    cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
	    exit(-BFL_ERRMISUSE);
	  }
	else{
	  cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
	  exit(-BFL_ERRMISUSE);
	}
      }
  }

}//namespace BFL

