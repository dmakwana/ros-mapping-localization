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
#include <Eigen/Dense>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <boost/random.hpp>
#include <math.h>
#include <algorithm>    // std::lower_bound, std::upper_bound, std::sort
#include <vector>       // std::vector
#include <iterator>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define IPS_TO_METERS 2.2

using namespace Eigen;
using namespace std;
using boost::math::normal; // typedef provides default type is double.

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

#define PI 3.14159
#define NUM_PARTICLES 25
#define STANDARD_DEVIATION 0.1

double particles[NUM_PARTICLES][3];
double previous_theta = 0;
double previous_time = 0;

double ips_x = 0;
double ips_y = 0;
double ips_yaw = 0;

typedef boost::mt19937                     ENG;    // Engine: Mersenne Twister
typedef boost::normal_distribution<double> DIST;   // Normal Distribution
typedef boost::variate_generator<ENG,DIST> GEN;    // Variate generators


// double std_dev = 0.1; // 10cm = 0.1m
#define STD_MEAN 0.0
// #define STD_DEV 0.1
#define SCALE_FACTOR 2.2

ENG  eng;
DIST dist(STD_MEAN, STANDARD_DEVIATION);
GEN  noise_gen(eng,dist);

// Normal Distribution 
// boost::mt19937 rng; // I don't seed it on purpouse (it's not relevant)
// boost::normal_distribution<> nd(0.0, STANDARD_DEVIATION);
// boost::variate_generator<boost::mt19937&,
//                         boost::normal_distribution<> > var_nor(rng, nd);

void measure_update(double ips_x, double ips_y, double ips_yaw) {

    double re_sampled[NUM_PARTICLES][3];
    vector<double> weights(NUM_PARTICLES);

    for(int i = 0; i < NUM_PARTICLES; i++) {
        double prev_weight = i ? weights.at(i - 1) : 0;
        normal x(particles[i][0], STANDARD_DEVIATION);
        normal y(particles[i][1], STANDARD_DEVIATION);
        normal yaw(particles[i][2], STANDARD_DEVIATION);
        weights.at(i) = boost::math::pdf(x, ips_x) *
                        boost::math::pdf(y, ips_y) *
                        boost::math::pdf(yaw, ips_yaw) + prev_weight;
        //ROS_INFO("prev_weight: %f Weight1: %f Weight2: %f Weight3: %f", prev_weight, boost::math::pdf(x, ips_x), boost::math::pdf(y, ips_y), boost::math::pdf(yaw, ips_yaw));


    }
    //for (int i = 0; i < NUM_PARTICLES; ++i) {
        //ROS_INFO("Weight: %f POSE: X: %f Y: %f", weights.at(i), particles[i][1], particles[i][2]);
    //}

    //transform(weights.begin(), weights.end(), weights.begin(),
              //std::bind1st(std::divides<double>(),weights.at(NUM_PARTICLES - 1)));

    for (int i = 0; i < NUM_PARTICLES; ++i) {
        weights.at(i) /= weights.at(NUM_PARTICLES - 1);
        //ROS_INFO("Particle: %d Weight: %f POSE: X: %f Y: %f", i, weights.at(i), particles[i][1], particles[i][2]);
    }
    //exit(-1);
    //ROS_INFO("Actual: POSE: X: %f Y: %f", ips_x, ips_y);
    //exit(-1);
    vector<double>::iterator low;
    for(int i = 0; i < NUM_PARTICLES; i++) {
        double random_num = (double)((double)(rand() % 10000) / 10000);
        low = lower_bound(weights.begin(), weights.end(), random_num);
        int index = (int)(low - weights.begin());
        //ROS_INFO("Random: %f LOW: %d ", random_num, index);
        re_sampled[i][0] = particles[index][0];
        re_sampled[i][1] = particles[index][1];
        re_sampled[i][2] = particles[index][2];
    }
    memcpy(particles, re_sampled, NUM_PARTICLES*3*sizeof(double));
}

//Callback function for the Position topic (SIMULATION)
// void pose_callback(const gazebo_msgs::ModelStates& msg)
// void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
// {
//     // int i;
//     // for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

//     // ips_x = msg.pose[i].position.x ;
//     // ips_y = msg.pose[i].position.y ;
//     // ips_yaw = tf::getYaw(msg.pose[i].orientation);
//     // if (ips_yaw < 0){
//     //     ips_yaw += (2 * PI);
//     // } else if (particles[i][2] > (2 * PI)) {
//     //     ips_yaw -= (2 * PI);
//     // }
//     ips_x = IPS_TO_METERS*msg.pose.pose.position.x; // Robot X psotition
//     ips_y = IPS_TO_METERS*msg.pose.pose.position.y; // Robot Y psotition
//     ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
//     ROS_DEBUG("pose_callback ips_x: %f ips_y: %f ips_yaw: %f", ips_x, ips_y, ips_yaw);
//     //ROS_INFO("POSE: X: %f Y: %f Yaw: %f",ips_x,ips_y,ips_yaw);
//     if (ips_yaw < 0){
//         ips_yaw += (2 * PI);
//     }

//     measure_update(ips_x + noise_gen(), ips_y + noise_gen(), ips_yaw + noise_gen());
// }

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{
    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
    if (ips_yaw < 0){
        ips_yaw += (2 * PI);
    } else if (particles[i][3] > (2 * PI)) {
        ips_yaw -= (2 * PI);
    }
    //ROS_INFO("POSE: X: %f Y: %f Yaw: %f",ips_x,ips_y,ips_yaw);
    measure_update(ips_x + noise_gen(), ips_y + noise_gen(), ips_yaw + noise_gen());
}

void prediction_update(double v, double omega, double dt) {
    //ROS_INFO("V: %f omega: %f dt: %f",v,omega,dt);
    for(int i = 0; i <  NUM_PARTICLES; i++) {
        particles[i][0] = particles[i][0] + v * cos(particles[i][2]) * dt + noise_gen();
        particles[i][1] = particles[i][1] + v * sin(particles[i][2]) * dt + noise_gen();
        particles[i][2] = particles[i][2] + omega + noise_gen();

        if (particles[i][2] < 0){
            particles[i][2] += (2 * PI);
        } else if (particles[i][2] > (2 * PI)) {
            particles[i][2] -= (2 * PI);
        }
    }
}

void odom_callback(const nav_msgs::Odometry& msg)
{   
    double stamp = msg.header.stamp.toSec();
    double dt = stamp - previous_time;
    //ROS_INFO("prev_time: %f new_tme: %f",previous_time, stamp);
    previous_time = stamp;

    double v = msg.twist.twist.linear.x;
    double theta = msg.twist.twist.angular.z;
    double omega = theta - previous_theta;
    previous_theta = theta;
    //TODO: remove y,z and add theta
    
    prediction_update(v, omega, dt);
}

void init_particles(){
    for(int i = 0; i < NUM_PARTICLES; i++){
        particles[i][0] =  (double)(((double)(rand() % 100)) / 10);
        particles[i][1] =  (double)(((double)(rand() % 100)) / 10);
        particles[i][2] =  (double)(((double)(rand() % 628318)) / 100000);
    }
}

int main(int argc, char **argv)
{ 
    init_particles();

	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    // ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
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
    	vel.linear.x = 1.0; // set linear speed
    	vel.angular.z = 0.3; // set angular speed

    	velocity_publisher.publish(vel); // Publish the command velocity

        visualization_msgs::Marker points, actuals;
        points.header.frame_id = "base_link";
        points.header.stamp = ros::Time::now();
        points.ns = "Points";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;

        points.id = 0;

        points.type = visualization_msgs::Marker::POINTS;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        actuals.header.frame_id = "base_link";
        actuals.header.stamp = ros::Time::now();
        actuals.ns = "actuals";
        actuals.action = visualization_msgs::Marker::ADD;
        actuals.pose.orientation.w = 1.0;

        actuals.id = 1;

        actuals.type = visualization_msgs::Marker::POINTS;
        // POINTS markers use x and y scale for width/height respectively
        actuals.scale.x = 0.2;
        actuals.scale.y = 0.2;

        // Points are green
        actuals.color.r = 1.0f;
        actuals.color.a = 1.0;

        geometry_msgs::Point actual_p;
        actual_p.x = ips_x;
        actual_p.y = ips_y;

        //ROS_INFO("Particle: %d POSE: X: %f Y: %f", i, p.x, p.y);
        actual_p.z = 0;

        actuals.points.push_back(actual_p);
        // Create the vertices for the points and lines
        for (int i = 0; i < NUM_PARTICLES; ++i) {

          geometry_msgs::Point p;
          p.x = particles[i][0];
          p.y = particles[i][1];
          //ROS_INFO("Particle: %d POSE: X: %f Y: %f", i, p.x, p.y);
          p.z = 0;

          points.points.push_back(p);
        }


        marker_pub.publish(points);
        marker_pub.publish(actuals);

    }

    return 0;
}
