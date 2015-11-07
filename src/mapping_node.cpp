//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Initial Authors (ME 597 TA's): James Servos & Nima Mohajerin
// Author: Digvijay Makwana
// University of Waterloo
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

// CONFIGURATION VARIABLES
#define PI 3.14159
#define ROWS 100 //10 meters with resolution of 0.1
#define COLS 100 //10 meters with resolution of 0.1
#define CELLS_PER_METER 10

const float g_resolution = (float)1/CELLS_PER_METER; // meters/cell
#define LOW_PROB 40
#define MID_PROB 50
#define HIGH_PROB 60
#define FULL_PROB 100

// PROTOTYPES
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y);

// GLOBAL VARIABLES
nav_msgs::OccupancyGrid grid;
std::vector<int8_t> myMap;
geometry_msgs::Pose initial_pose;
ros::Time map_load_time;
bool initial_pose_found = false;

// bool new_scan_data = false;
sensor_msgs::LaserScan g_scan_data;

using std::endl;
using std::cout;

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double init_ips_x;
double init_ips_y;
double init_ips_yaw;
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

    if (!initial_pose_found) {
        init_ips_x = ips_x;
        init_ips_y = ips_y;
        init_ips_yaw = ips_yaw;
        initial_pose_found = true;
    }

}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_INFO("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

int8_t get_prob_from_logit(float logit) {
    float exp_logit = exp(logit);
    return (int8_t)FULL_PROB*(exp_logit / (1 + exp_logit));
}

float logit(int prob) {
    return log(((float)prob/(FULL_PROB-prob)));
}

//Callback function for the scan
void scan_callback(const sensor_msgs::LaserScan& msg) {
    // ROS_INFO("scan_callback!");
    int i = 0, x0 = 0, y0 = 0, x1 = 0, y1 = 0, j = 0, r = 0;
    float angle;
    float actual_angle;
    // float *values = &msg.ranges[0];
    // ROS_INFO("AT 0 is: %f", values[0]);
    // ROS_INFO("Number of Data Points %d!", (int)msg.ranges.size());
    for (i = 0; i < (int)msg.ranges.size(); i++) {
        // ROS_INFO("%d: %f", i, msg.ranges.at(i));
        // ROS_INFO("sup");
        angle = msg.angle_min + i * msg.angle_increment;
        actual_angle = ips_yaw - angle;
        // Update cells for where we have data
        // ROS_INFO("sup0 %f, %f", msg.range_min, msg.range_max);
        if (isnan(msg.ranges.at(i)) || msg.ranges.at(i) > msg.range_max || msg.ranges.at(i) < msg.range_min) { continue; }
        x0 = CELLS_PER_METER*(ips_x - init_ips_x) + (COLS-1)/2;
        y0 = CELLS_PER_METER*(ips_y - init_ips_y) + (ROWS-1)/2;
        x1 = x0 + CELLS_PER_METER * msg.ranges[i] * cos(actual_angle);
        y1 = y0 + CELLS_PER_METER * msg.ranges[i] * sin(actual_angle);
        std::vector<int> x, y;
        // ROS_INFO("INITIAL IPS_YAW: %f", init_ips_yaw*180/PI);
        // ROS_INFO("IPS_YAW: %f", ips_yaw*180/PI);
        // ROS_INFO("PHI: %f", angle*180/PI);
        // ROS_INFO("ACTUAL ANGLE: %f", actual_angle*180/PI);
        // ROS_INFO("RANGE: %f", msg.ranges.at(i));
        // ROS_INFO("sup1 %f, %f, %f", cos(actual_angle),sin(actual_angle), (float)msg.ranges.at(i));
        // ROS_INFO("sup2 %d, %d, %d, %d", x0 ,y0, x1, y1);
        // ROS_INFO("sup3 yaw: %f, range: %f, phi %f,", ips_yaw ,msg.ranges.at(i), angle);
        bresenham(x0, y0, x1, y1, x, y);
        // ROS_INFO("sup2");
        // exit(-1);    
        // ROS_INFO("vector_size: %d", (int)x.size());
        for (int j = 0; j < x.size(); j++) {
            // ROS_INFO("HERE2");
            int x_delta = x[j]-x0;
            int y_delta = y[j]-y0;
            r = sqrt(x_delta*x_delta + y_delta*y_delta);
            // ROS_INFO("R: %d", r);
            // ROS_INFO("msg.range: %f", msg.ranges.at(i));
            // ROS_INFO("%d, %d", x[j], y[j]);
            int new_prob;
            if (r < (msg.ranges.at(i) * 10 - 2)) { 
                new_prob = LOW_PROB;
                // ROS_INFO("LOW_PROB");
            } 
            else { 
                // ROS_INFO("HIGH_PROB");
                new_prob = HIGH_PROB; 
            }
            // ROS_INFO("HERE");
            int old_prob = myMap[y[j]*COLS + x[j]];
            // if (old_prob != 0) {
            //     ROS_INFO("NOT ZERO");
            // } else {
            //     ROS_INFO("ZERO");
            //     exit(-1);
            // }
            // ROS_INFO("OLD PROB AT %d, %d: %d", x[j], y[j], old_prob);
            // ROS_INFO("OLD LOGIT: %f, NEW_LOGIT: %f", logit(old_prob), logit(new_prob));
            // ROS_INFO("UPDATING_PROB: %d", new_prob); 
            
            float new_logit = logit(new_prob) + logit(old_prob);
            myMap[y[j]*COLS + x[j]] = get_prob_from_logit(new_logit);
            int m = 1 + myMap[y[j]*COLS + x[j]];
            // ROS_INFO("NEW PROB: %d", m);
        }
    }
    // ROS_INFO("scan_callback finished!");
    return;
}
//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber scan_sub = n.subscribe("/scan", 1, scan_callback);
    
    //Setup topics to Publish from this node
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    ROS_INFO("Subscribed to everything!");
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    // Occupany Grid Map and MapMetaData variable
    nav_msgs::MapMetaData metadata;

    ROS_INFO("Waiting for initial pose!");

    ros::Rate loop_rate(10);    //20Hz update rate
    while (!initial_pose_found) {
        loop_rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("Received initial pose!");

    metadata.origin = initial_pose;
    metadata.map_load_time = ros::Time::now();
    grid.info = metadata;
    grid.header.frame_id = "base_footprint";

    ROS_INFO("Initializing grid cells!");
    for (int i = 0; i < ROWS*COLS; i++) {
        myMap.push_back(MID_PROB);
    }
    grid.data = myMap;
    grid.header.seq = 0;
    ROS_INFO("Initialized grid cells!");
    //Set the loop rate
	
    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

    	//Main loop code goes here:
    	vel.linear.x = 0.1; // set linear speed
    	vel.angular.z = 0.3; // set angular speed

        metadata.resolution = g_resolution;
        metadata.width = COLS;
        metadata.height = ROWS;

        grid.info = metadata;
        grid.data = myMap;
        grid.header.stamp = ros::Time::now();

    	velocity_publisher.publish(vel); // Publish the command velocity     
        map_publisher.publish(grid);
        grid.header.seq++;
    }

    return 0;
}
