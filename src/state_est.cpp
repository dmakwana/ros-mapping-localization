//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <boost/random.hpp>

using namespace Eigen;
using namespace std;
using boost::math::normal; // typedef provides default type is double.

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double ips_x;
double ips_y;
double ips_yaw;

short sgn(int x) { return x >= 0 ? 1 : -1; }

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{
    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
    // ROS_INFO("POSE: X: %f Y: %f Yaw: %f",ips_x,ips_y,ips_yaw);
}

void odom_callback(const nav_msgs::Odometry& msg)
{
  double v = msg.twist.twist.linear.x;
  double theta = msg.twist.twist.angular.z;

  //TODO: remove y,z and add theta

  if ( v > 0.01 || theta > 0.01 ) {
    ROS_INFO("ODOMETRY V: %f, Theta: %f",v,theta);
  }

}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

void meas_mod(){
    Matrix3f A;
    A <<  1, 0, 0,
          0, 1, 0,
          0, 0, 1;

    Matrix3f B;
    cout << A;
}

int main(int argc, char **argv)
{
    // Normal Distribution Example
    boost::mt19937 rng; // I don't seed it on purpouse (it's not relevant)
    boost::normal_distribution<> nd(0.0, 0.1);
    boost::variate_generator<boost::mt19937&,
                            boost::normal_distribution<> > var_nor(rng, nd);

    for(int i = 0; i < 10; ++i) {
        double d = var_nor();
        cout << "Random = " << d << endl;
    }
    // End of Example

    meas_mod();
    // Eigen Example Code
    Matrix3d m = Matrix3d::Random();
    m = (m + Matrix3d::Constant(1.2)) * 50;

    cout << "m =" << endl << m << endl;
    Vector3d v(1,2,3);

    cout << "m * v =" << endl << m * v << endl;
    // End of example

	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
	  ROS_INFO("STARTING NODE LOGGING");

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

    	//Main loop code goes here:
    	vel.linear.x = 0.1; // set linear speed
    	vel.angular.z = 0.3; // set angular speed

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
