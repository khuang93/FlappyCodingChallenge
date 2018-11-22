#ifndef FLAPPY_AUTOMATION_CODE_H_
#define FLAPPY_AUTOMATION_CODE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"

//Ros nodehandle
ros::NodeHandle* nh_= NULL;
//Publisher for acceleration command
ros::Publisher pub_acc_cmd;
//Subscriber for velocity
ros::Subscriber sub_vel;
//Subscriber for laser scan
ros::Subscriber sub_laser_scan;

struct Point{
  float x; //in body coordinate, distance from flappy
  float y; //in absolute coordinate
  int type; //0 is obstacle, 1 is wall
  Point(float _x, float _y):x(_x),y(_y){ 
      type = 0;
  }
};



void initNode();
void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void convertLaserScan2PCL(std::vector<Point>& out, std::vector<float> ranges, float range_max, float range_min, float angle_min, float angle_max, float angle_increment, int number_laser_rays);
bool isValidPoint(float range, float range_max, float range_min);
void updatePCL();
void updateFlappyPos();

#endif
