// Jarvis Schultz
// November 10, 2011

// prototypes for a nonlinear analytic conditional gaussian model for
// a mobile robot.  This was adapted from an example program on the
// orocos/ bfl website
// http://people.mech.kuleuven.be/~tdelaet/bfl_doc/getting_started_guide/

#ifndef __NON_LINEAR_SYSTEM_CONDITIONAL_GAUSSIAN_MOBILE__
#define __NON_LINEAR_SYSTEM_CONDITIONAL_GAUSSIAN_MOBILE__

#include <pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{
  /// Non Linear Analytic Conditional Gaussian Model for Mobile Robots

  class NonLinearAnalyticConditionalGaussianMobile :
    public AnalyticConditionalGaussianAdditiveNoise
    {
    public:
      /// Constructor
      NonLinearAnalyticConditionalGaussianMobile( const Gaussian& additiveNoise);

      /// Destructor
      virtual ~NonLinearAnalyticConditionalGaussianMobile();

      // redefine virtual functions
      virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
      virtual MatrixWrapper::Matrix          dfGet(unsigned int i)       const;
    };

} // End namespace BFL

#endif //
