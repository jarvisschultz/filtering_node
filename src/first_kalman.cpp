// first_kalman.cpp
// Jarvis Schultz
// Fall 2011

//---------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// This package will test an implementation of an EKF.  It will
// subscribe to the inputs sent to the robot that are published by a
// puppeteer_control node and use these to form an expectation.  Then
// it will also subscribe to the estimates of the robot pose that are
// published by the Kinect, and it will use those as the measurement.
// Finally it will publish its own filtered estimate of the robot's
// pose.

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <iostream>

#include <ros/ros.h>
#include <puppeteer_msgs/RobotPose.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/position_request.h>
#include <puppeteer_msgs/speed_command.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

// Kalman filter stuff:
#include "../include/mobile_robot.h"
#include "../include/nonlinearanalyticconditionalgaussianmobile.h"
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

//---------------------------------------------------------------------------
// Global Variables and #defines
//---------------------------------------------------------------------------
#define NUM_STATES	(3)
#define NUM_INPUTS	(2)
#define KIN_COV_DIST	(0.0025)
#define KIN_COV_ORI	(0.2)
#define SYS_COV_DIST	(0.001)
#define SYS_COV_ORI	(0.01)


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------
class FilterGenerator {

private:
    ros::NodeHandle node_;
    ros::Publisher est_pub;
    ros::Timer timer;
    ros::Time t_now, t_last, tstamp;
    ros::Subscriber input_sub, kin_sub;
    nav_msgs::Odometry kin_pose, est_pose;
    tf::TransformListener tf;
    tf::TransformBroadcaster br;
    ColumnVector sys_noise_mu, prior_mu, input, measurement, meas_noise_mu;
    SymmetricMatrix sys_noise_cov, prior_cov, meas_noise_cov;
    // Gaussian system_uncertainty, measurement_uncertainty;
    Gaussian* prior_cont;
    NonLinearAnalyticConditionalGaussianMobile* sys_pdf;
    AnalyticSystemModelGaussianUncertainty* sys_model;
    LinearAnalyticConditionalGaussian* meas_pdf;
    LinearAnalyticMeasurementModelGaussianUncertainty* meas_model;
    ExtendedKalmanFilter* filter;
    MobileRobot mobile_robot;

public:
    // Constructor
    FilterGenerator() {
	// Setup ROS stuff:
	kin_sub = node_.subscribe("/vo", 10, &FilterGenerator::kinectcb, this);
	input_sub = node_.subscribe
	    ("/serviced_values", 10, &FilterGenerator::inputcb, this);
	timer = node_.createTimer(ros::Duration(0.033), &FilterGenerator::timercb, this);

	// Initialize misc variables:
	tstamp = ros::Time::now();



	//********************//
	// Setup system model:
	//********************//
	ColumnVector sys_noise_mu(NUM_STATES);
	sys_noise_mu(1) = 0.0;
	sys_noise_mu(2) = 0.0;
	sys_noise_mu(3) = 0.0;

	SymmetricMatrix sys_noise_cov(NUM_STATES);
	sys_noise_cov = 0.0;
	sys_noise_cov(1,1) = SYS_COV_DIST;
	sys_noise_cov(2,2) = SYS_COV_DIST;
	sys_noise_cov(3,3) = SYS_COV_ORI;

	// Create system gaussian:
	Gaussian system_uncertainty(sys_noise_mu, sys_noise_cov);

	// Create system model:
	sys_pdf = new BFL::NonLinearAnalyticConditionalGaussianMobile(system_uncertainty);
	sys_model = new BFL::AnalyticSystemModelGaussianUncertainty(sys_pdf);


	//************************//
	// Setup measurement model:
	//************************//
	SymmetricMatrix Hmat(NUM_STATES);
	Hmat = 0.0;
	Hmat(1,1) = 1;
	Hmat(2,2) = 1;
	Hmat(3,3) = 1;

	ColumnVector meas_nose_mu(NUM_STATES);
	meas_noise_mu(1) = 0.0;
	meas_noise_mu(2) = 0.0;
	meas_noise_mu(3) = 0.0;

	SymmetricMatrix meas_noise_cov(NUM_STATES);
	meas_noise_cov = 0.0;
	meas_noise_cov(1,1) = KIN_COV_DIST;
	meas_noise_cov(2,2) = KIN_COV_DIST;
	meas_noise_cov(3,3) = KIN_COV_ORI;

	// create measurement gaussian:
	Gaussian measurement_uncertainty(meas_noise_mu, meas_noise_cov);
	
	// create system model:
	meas_pdf = new BFL::LinearAnalyticConditionalGaussian(
	    Hmat, measurement_uncertainty);
	meas_model = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf);


	//******************************//
	// Setup initial parameters:
	//******************************//


	// Create filter:
	
			
	// Now we are ready to run the filter:
	ROS_INFO("Starting Kalman Estimator...");
    }

    // // Destructor
    // ~KalmanFilter(){
    // 	delete timer;
    // 	delete input_sub;
    // 	delete kin_sub;
    // }

    // In this callback, let's just update the current measurement 
    void kinectcb(const nav_msgs::Odometry pose)
	{
	    // measurement(1) = 
	    return;
	}

    // in this callback, let's update the local values of the inputs
    void inputcb(const geometry_msgs::PointStamped)
	{
	    return;
	}

    // in this callback, let's integrate the model forward to get an
    // expectation, then run the filter forward in time, and then
    // publish the results of the filter as well as a tf from /map to
    // a robot frame
    void timercb(const ros::TimerEvent& e)
	{
	    return;
	}

};// End class


int main(int argc, char **argv)
{

    ros::NodeHandle node;

    FilterGenerator fg;

    ros::spin();

    return 0;
}


