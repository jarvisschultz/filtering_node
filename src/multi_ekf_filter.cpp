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
#include "log4cxx/logger.h"
#include <angles/angles.h>

#include <math.h>

// Kalman filter stuff:
#include "../include/mobile_robot.h"
#include "../include/filter_constants.h"
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
#define KIN_COV_DIST	(0.5)
#define KIN_COV_ORI	(100.0)
#define SYS_COV_DIST	(0.001)
#define SYS_COV_ORI	(0.01)
#define FILTER_TIMEOUT	(1.0)
#define COV_MULTIPLIER  (10000.0)
#define WHEEL_DIA (0.07619999999999)
#define WIDTH (0.148/2.0)
#define ANGLE_TOLERANCE (0.1*M_PI)

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------
class FilterGenerator {

private:
    ros::NodeHandle node_;
    ros::Publisher est_pub;
    ros::Time  tstamp; 
    ros::Subscriber input_sub, kin_sub;
    nav_msgs::Odometry est_pose;
    tf::TransformListener tf;
    tf::TransformBroadcaster br;
    geometry_msgs::PointStamped current_command, last_command;
    nav_msgs::Odometry current_measurement, last_measurement;
    ColumnVector sys_noise_mu, prior_mu, measurement, meas_noise_mu, input;
    SymmetricMatrix sys_noise_cov, prior_cov, meas_noise_cov;
    Gaussian* prior_cont;
    NonLinearAnalyticConditionalGaussianMobile* sys_pdf;
    AnalyticSystemModelGaussianUncertainty* sys_model;
    LinearAnalyticConditionalGaussian* meas_pdf;
    LinearAnalyticConditionalGaussian* bad_meas_pdf;
    LinearAnalyticMeasurementModelGaussianUncertainty* meas_model;
    LinearAnalyticMeasurementModelGaussianUncertainty* bad_meas_model;
    ExtendedKalmanFilter* filter;
    MobileRobot* mobile_robot;
    char ns;
    
public:
    // Constructor
    FilterGenerator() {
	// Print the start of the file...
	ROS_INFO("Starting Kalman Estimator...");

	ROS_DEBUG("Creating subscribers, and publishers");
	// Setup ROS stuff:
	kin_sub = node_.subscribe("vo", 100,
				  &FilterGenerator::measurementcb, this);
	input_sub = node_.subscribe
	    ("serviced_values", 100, &FilterGenerator::inputcb, this);
	est_pub = node_.advertise<nav_msgs::Odometry> ("pose_ekf", 100);
	
	// Initialize misc variables:
	tstamp = ros::Time::now();
	// get namespace
	std::string name = ros::this_node::getNamespace();
	if (!name.empty())
	    ns = *name.rbegin();
	else
	    ns = '0';

	// initialize all filter paramters:
	InitializeFilter();			
    }
    

    // Destructor
    ~FilterGenerator(){
    	delete prior_cont;
    	delete filter;
    	delete sys_pdf;
    	delete sys_model;
    	delete meas_pdf;
    	delete meas_model;
    	delete bad_meas_pdf;
	delete bad_meas_model;
    }


    
    void InitializeFilter(void)
	{
	    ROS_DEBUG("Initializing all EKF parameters");
	    

	    /////////////////////////
            // Setup system model: //
            /////////////////////////
	    ROS_DEBUG("Defining system model");
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
	    ROS_DEBUG("Creating system model");
	    sys_pdf = new BFL::
		NonLinearAnalyticConditionalGaussianMobile(system_uncertainty);
	    sys_model = new BFL::AnalyticSystemModelGaussianUncertainty(sys_pdf);
 


	    //////////////////////////////
            // Setup measurement model: //
            //////////////////////////////
	    ROS_DEBUG("Defining measurement model");
	    Matrix Hmat(NUM_STATES, NUM_STATES);
	    Hmat = 0.0;
	    Hmat(1,1) = 1;
	    Hmat(2,2) = 1;
	    Hmat(3,3) = 1;

	    ColumnVector meas_noise_mu(NUM_STATES);
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
	
	    // create measurement model:
	    ROS_DEBUG("Creating measurement model");
	    meas_pdf = new BFL::LinearAnalyticConditionalGaussian(
		Hmat, measurement_uncertainty);
	    meas_model = new BFL::
		LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf);

	    /////////////////////////////////
            // Setup bad measurement model //
            /////////////////////////////////
	    meas_noise_cov(1,1) = COV_MULTIPLIER*KIN_COV_DIST;
	    meas_noise_cov(2,2) = COV_MULTIPLIER*KIN_COV_DIST;
	    meas_noise_cov(3,3) = COV_MULTIPLIER*KIN_COV_ORI;

	    // create bad measurement gaussian:
	    Gaussian bad_measurement_uncertainty(meas_noise_mu, meas_noise_cov);
	    bad_meas_pdf = new BFL::LinearAnalyticConditionalGaussian(
		Hmat, bad_measurement_uncertainty);
	    bad_meas_model = new BFL::
		LinearAnalyticMeasurementModelGaussianUncertainty(bad_meas_pdf);
	    
	    
	    ///////////////////////////////
            // Instantiate a MobileRobot //
            ///////////////////////////////
	    // First get the initial parameters published by the control node
	    ROS_DEBUG("Creating a MobileRobot");
	    ColumnVector init(STATE_SIZE);
	    init = 0.0;
	    if(ros::param::has("robot_x0"))
	    {
		// Get robot's starting position in
		// optimization coordinate system
		double temp;
		ros::param::get("robot_x0", temp);
		init(1) = temp;
		ros::param::get("robot_z0", temp);
		init(2) = -temp;
		ros::param::get("robot_th0", temp);

		temp = angles::normalize_angle(-temp);
		init(3) = temp;
		// Initialize robot:
		mobile_robot = new MobileRobot(init);
	    }
	    else
	    {
		ROS_ERROR("NO STARTING INFORMATION:"\
			  "Must run control node before starting this node!");
		exit(0);
	    }

	    // Get inputs and measurements initialized:
	    ROS_DEBUG("Defining input size");
	    input.resize(NUM_INPUTS);
	    measurement.resize(NUM_STATES);
	    input(1) = 0;
	    input(2) = 0;
		      
	    ///////////////////////////////
            // Setup initial parameters: //
            ///////////////////////////////
	    ROS_DEBUG("Defining filter parameters");
	    ColumnVector prior_mu(STATE_SIZE);
	    for (unsigned int i = 1; i<=STATE_SIZE; i++)
		prior_mu(i) = init(i);
	    SymmetricMatrix prior_cov(STATE_SIZE);
	    prior_cov = 0;
	    prior_cov(1,1) = KIN_COV_DIST;
	    prior_cov(2,2) = KIN_COV_DIST;
	    prior_cov(3,3) = KIN_COV_ORI;

	    prior_cont = new Gaussian(prior_mu, prior_cov);
		
	    // Create filter:
	    ROS_DEBUG("Creating filter");
	    filter = new ExtendedKalmanFilter(prior_cont);
	}

    

    // In this callback, let's update the current measurement, check
    // angles, then update the filter, then publish the results.
    void measurementcb(const nav_msgs::Odometry p)
	{
	    ROS_DEBUG("EKF measurement callback triggered");
	    int operating_condition = 0;
	    bool reset = false;
	    // check out if we need to reset the filter parameters:
	    ros::param::getCached("/operating_condition", operating_condition);
	    reset = reset_logic(operating_condition);
	    if (reset)
	    {
		ROS_WARN("Resetting filter");
		InitializeFilter();
		return;
	    }

	    if (operating_condition == 1 || operating_condition == 2)
	    {
		static bool first_flag = true;
		if (first_flag)
		{
		    last_measurement.header.stamp = ros::Time::now();
		    current_measurement.header.stamp = ros::Time::now();
		    first_flag = false;
		    return;
		}

		ROS_DEBUG("Filling out measurement values");
		measurement(1) = p.pose.pose.position.x;
		measurement(2) = p.pose.pose.position.y;
		double theta = tf::getYaw(p.pose.pose.orientation);
		theta = angles::normalize_angle(theta);
		measurement(3) = theta;
		
		ROS_DEBUG("Storing measurements");
		last_measurement = current_measurement;
		current_measurement = p;

		// check for bad measurements (occlusion?)
		LinearAnalyticMeasurementModelGaussianUncertainty* meas =
		    new LinearAnalyticMeasurementModelGaussianUncertainty();
		if (p.pose.covariance[0] < 10.0)
		    meas = meas_model;
		else
		{
		    ROS_WARN_THROTTLE(1,
				      "Huge covariance detected for robot %c",ns);
		    meas = bad_meas_model;
		    measurement = 0;
		}

		ROS_DEBUG("Checking for filter timeout");
		// check for timeout:
		double dt = (p.header.stamp -
			     last_measurement.header.stamp).toSec();
		if (dt >= FILTER_TIMEOUT)
		{
		    ROS_WARN("Filter timeout detected (%f, %f)",
			     p.header.stamp.toSec(),
			     last_measurement.header.stamp.toSec());
		    first_flag = true;
		    return;
		}

		// Integrate the model forward in time:
		ROS_DEBUG("Integrating model forward in time");
		if(dt >= 0)
		    mobile_robot->Move(input*dt);
		else
		    ROS_WARN("Negative dt when integrating system kinematics");

		ROS_DEBUG("Updating the filter");

		// Now correct measured angle to be consistent with the estimate:
		ColumnVector s = mobile_robot->GetState();
		measurement(3) = s(3) - angles::shortest_angular_distance(
		    measurement(3), s(3));

		// Now we are ready to update the filter:
		filter->Update(sys_model, input*dt, meas, measurement);
	    }

	    // Fill out the message to publish:
	    ROS_DEBUG("Extracting and publishing posterior estimate");
	    Pdf<ColumnVector> * posterior = filter->PostGet();
	    ColumnVector curr_state = posterior->ExpectedValueGet();
	    curr_state(3) = angles::normalize_angle(curr_state(3));
	    
	    std::stringstream ss;
	    ss << "robot_" << ns << "_base_footprint_mine";

	    est_pose.header.stamp = p.header.stamp;
	    est_pose.header.frame_id = "/map";
	    est_pose.child_frame_id = ss.str();
	    est_pose.pose.pose.position.x = curr_state(1);
	    est_pose.pose.pose.position.y = curr_state(2);
	    est_pose.pose.pose.position.z = 0.0;

	    geometry_msgs::Quaternion quat =
	    	tf::createQuaternionMsgFromYaw(curr_state(3));
	    est_pose.pose.pose.orientation = quat;

	    ROS_DEBUG("Publishing EFK Pose");
	    est_pub.publish(est_pose);

	    // now, let's publish the transform that goes along with it
	    ROS_DEBUG("Filling out tf message");
	    geometry_msgs::TransformStamped est_trans;
	    tf::Quaternion q1, q2;
	    q1 = tf::createQuaternionFromYaw(curr_state(3));
	    q2 = tf::Quaternion(1.0,0,0,0);
	    q1 = q1*q2;
	    tf::quaternionTFToMsg(q1, quat);
	    	    
	    est_trans.header.stamp = p.header.stamp;
	    est_trans.header.frame_id = est_pose.header.frame_id;
	    est_trans.child_frame_id = est_pose.child_frame_id;
	    est_trans.transform.translation.x = est_pose.pose.pose.position.x;
	    est_trans.transform.translation.y = est_pose.pose.pose.position.y;
	    est_trans.transform.translation.z = est_pose.pose.pose.position.z;
	    est_trans.transform.rotation = quat;

	    ROS_DEBUG("Sending transform for the output of the EKF filter");
	    br.sendTransform(est_trans);

	    return;
	}


    // in this function, we will determine if we need to reset the
    // kalman filter:
    bool reset_logic(int op)
	{
	    bool reset = false;
	    static int op_old = 0;

	    // if we are in calibrate, and we weren't previously,
	    // let's reset the filter:
	    if (op == 1 && op_old != 1)
		reset = true;
	    // if we are in run, and we were not previously, let's reset:
	    else if (op == 2 && op_old != 2 && op_old != 1)
		reset = true;
	    
	    op_old = op;
	    return(reset);
	}

    

    // in this callback, let's update the local values of the inputs
    void inputcb(const geometry_msgs::PointStamped &sent)
	{
	    static bool first_flag = true;
	    char type = (char) (sent.header.frame_id.c_str())[0];
	    bool valid = true;
	    ROS_DEBUG("EKF input callback triggered");
	    switch(type)
	    {
	    case 'd':
		input(1) = sent.point.x;
		input(2) = sent.point.y;
		break;
	    case 'i':
		input(1) = sent.point.x;
		input(2) = sent.point.y;
		break;
	    case 'h':
		input(1) = WHEEL_DIA*(sent.point.x+sent.point.y)/4.0;
		input(2) = WHEEL_DIA*(sent.point.y-sent.point.x)/(4.0*WIDTH);
		break;
	    case 'n':
		input(1) = WHEEL_DIA*(sent.point.x+sent.point.y)/4.0;
		input(2) = WHEEL_DIA*(sent.point.y-sent.point.x)/(4.0*WIDTH);
		break;
	    default :
		input(1) = 0;
		input(2) = 0;
		valid = false;
		break;
	    }
	    if (valid)
	    {
		last_command = current_command;
		current_command = sent;

		if (first_flag)
		{
		    first_flag = false;
		    return;
		}
		
		double dt = (sent.header.stamp
			     -last_command.header.stamp).toSec();
		if (dt >= FILTER_TIMEOUT)
		{
		    ROS_WARN("It has been longer than %f" \
			     "s since a command was published (%f, %f)",
			     dt, sent.header.stamp.toSec(),
			     last_command.header.stamp.toSec());
		    first_flag = true;
		}
	    }
	    ROS_DEBUG("Current Input values are v = %f w = %f",input(1), input(2));
	   
	    return;
	}


    // in this function, we take in two angles, and using one as the
    // reference, we keep adding and subtracting 2pi from the other to
    // find the mininmum angle between the two:
    void angle_correction(double& a, double& ref)
	{
	    while ((a-ref) > M_PI) a -= 2*M_PI;
	    while ((a-ref) < -M_PI) a += 2*M_PI;
	}

    
};// End class


int main(int argc, char **argv)
{
    ROSCONSOLE_AUTOINIT;
    
    ros::init(argc, argv, "pose_ekf");

    // log4cxx::LoggerPtr my_logger =
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle node;

    FilterGenerator fg;

    ros::spin();

    return 0;
}
