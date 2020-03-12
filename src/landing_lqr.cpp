#include <iostream>
#include <cmath>
#include <cstdlib>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "chrono"
#include "NumCpp.hpp"
#include <boost/filesystem.hpp>

using namespace std;

auto t2 = chrono::high_resolution_clock::now();

double yd;
double Ts = 10;
float theta, phi, psi;

ros::Publisher velPub;
ros::ServiceClient stateClient;
gazebo_msgs::ModelStates S;
tf2::Quaternion _quat;
geometry_msgs::Quaternion quat;
geometry_msgs::Point positionObj;
geometry_msgs::Pose kwadPose;
geometry_msgs::Twist twistObj;
gazebo_msgs::ModelState kwadModelState;
gazebo_msgs::SetModelState srv;
std_msgs::Float64MultiArray f;

nc::NdArray<double> K = {{0,0,0,0,0,0,0,0,0.3539,0,0,0.0447},{0.3787, 0,0,0.1946, 0,0,0, -0.0359, 0,0,-0.0141,0},{0, 0.3787, 0, 0, 0.1946, 0, -0.0359, 0, 0, -0.0141, 0, 0}, {0,0,0.0141, 0,0,0.0825,0,0,0,0,0,0}};
//::NdArray<double> K = {{0,0,0,0,0,0,0,0,0.9415,0,0,0.3162},{1.5857, 0,0,0.3995, 0,0,0, -0.2, 0,0,-0.1,0},{0, 1.5857, 0, 0, 0.3995, 0, -0.2, 0, 0, -0.1, 0, 0}, {0,0,0.1, 0,0,0.2214,0,0,0,0,0,0}};

nc::NdArray<double> A = {{0,0,0,1,0,0,0,0,0,0,0,0},{0,0,0,0,1,0,0,0,0,0,0,0},{0,0,0,0,0,1,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0},{0,-9.81,0,0,0,0,0,0,0,0,0,0},{-9.81,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,1,0,0,0,0,0},{0,0,0,0,0,0,0,1,0,0,0,0},{0,0,0,0,0,0,0,0,1,0,0,0}};
        
nc::NdArray<double> B = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,20,0,0},{0,0,20,0},{0,0,0,4.1667},{0,0,0,0},{0,0,0,0},{0.7142,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  
nc::NdArray<double> Hc_inv = {{0,0,0,0,0,0,0,0,0,0,0,0.0447},{0,0,0,0,0,0,0,0,0,0,-0.0141,0},{0,0,0,0,0,0,0,0,0,-0.0141,0,0},{0,0,0,0,0,0,0,0,0,0,0,0}};        
//nc::NdArray<double> Hc_inv = {{0,0,0,0,0,0,0,0,0,0,0,0.3162},{0,0,0,0,0,0,0,0,0,0,-0.1,0},{0,0,0,0,0,0,0,0,0,-0.1,0,0},{0,0,0,0,0,0,0,0,0,0,0,0}};

nc::NdArray<double> x_vec1 = {0,0,0,0,0,0,0,0,0,20.0,18.0,15.0};

nc::NdArray<double> t = {0,0,0,0,0,0,0,0,0,0,0,0};

auto t_ = t.transpose();
auto x_vec = x_vec1.transpose();
auto x_dot = nc::zeros<double>(12, 1);

void init_states()
{
    if(abs(x_vec[11] - yd) < 0.00001)yd -= 0.5;
    
}

void control_kwad(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    auto t1 = chrono::high_resolution_clock::now();
    auto dt = chrono::duration_cast<chrono::milliseconds>(t1-t2).count();
    init_states();
    if(dt > Ts)
    {
        t_[11] = yd;
        x_dot = nc::dot<double>((A - nc::dot<double>(B, K)), x_vec) + nc::dot<double>(B, nc::dot<double>(Hc_inv, t_));
        
        x_vec = x_dot * (Ts/1000) + x_vec;
        if(x_vec[11] < 0)x_vec[11] = 0;
        t2 = t1;
    }
    
    _quat.setRPY(x_vec[1], x_vec[0], x_vec[2]);
	_quat.normalize();
	quat = tf2::toMsg(_quat);
	
	positionObj.x = x_vec[9];
	positionObj.y = x_vec[10];
	positionObj.z = x_vec[11];
	twistObj.linear.x = x_vec[7];
	twistObj.linear.y = x_vec[6];
	twistObj.linear.z = x_vec[8];
	twistObj.angular.x = x_vec[4];
	twistObj.angular.y = x_vec[3];
	twistObj.angular.z = x_vec[5];
	
	kwadPose.position = positionObj;
	kwadPose.orientation = quat;
	
	kwadModelState.model_name = (string)"Kwad";
	kwadModelState.pose = kwadPose;
	kwadModelState.twist = twistObj;
	
	//cout << x_vec;
	
	f.data = {20, -20, 20, -20};
	velPub.publish(f);
		
	srv.request.model_state = kwadModelState;
		
	if(!stateClient.call(srv))
    {
        ROS_ERROR("Failed to move Kwad Error msg:%s",srv.response.status_message.c_str());
    }
	
}  

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Control");
	ros::NodeHandle nodeObj;
	
	velPub = nodeObj.advertise<std_msgs::Float64MultiArray>("/Kwad/joint_motor_controller/command", 40);
	
	stateClient = nodeObj.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", 1000);
	
	ros::Subscriber posSub = nodeObj.subscribe("/gazebo/model_states", 10, control_kwad);
	
	ros::spin();
	
	return 0;
}

