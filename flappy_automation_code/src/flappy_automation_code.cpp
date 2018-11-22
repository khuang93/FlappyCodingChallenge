#include "ros/ros.h"
#include "flappy_automation_code/flappy_automation_code.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include <vector>
#include <math.h>




void initNode()
{
  //Initialization of nodehandle
  nh_ = new ros::NodeHandle();
  //Init publishers and subscribers
  pub_acc_cmd = nh_->advertise<geometry_msgs::Vector3>("/flappy_acc",1);
  sub_vel = nh_->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, velCallback);
  sub_laser_scan = nh_->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, laserScanCallback);
}

void velCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  // msg has the format of geometry_msgs::Vector3
  // Example of publishing acceleration command on velocity velCallback
  geometry_msgs::Vector3 acc_cmd;

  acc_cmd.x = 1;
  acc_cmd.y = 0;
  pub_acc_cmd.publish(acc_cmd);
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //msg has the format of sensor_msgs::LaserScan
  //print laser angle and range
 
  //ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);

  int number_laser_rays = (msg->angle_max-msg->angle_min)/msg->angle_increment + 1;
  ROS_INFO("Laser number_laser_rays: %i", number_laser_rays);
 
  Point flappyPos(0,0);
  std::vector<Point> pcl;
  convertLaserScan2PCL(pcl, msg->ranges, msg->range_max, msg->range_min, (float)msg->angle_min, (float)msg->angle_max, (float)msg->angle_increment, number_laser_rays);

   for(int i = 0; i < number_laser_rays; i++){
    ROS_INFO("Laser range: %f, angle: %f", msg->ranges[i], msg->angle_min+msg->angle_increment*i);

  }

  for(int i = 0; i < pcl.size(); i++){
      ROS_INFO("PCL x: %f, y: %f", pcl.at(i).x,  pcl.at(i).y);
  }

}

void convertLaserScan2PCL(std::vector<Point>& out, std::vector<float> ranges, float range_max, float range_min, float angle_min, float angle_max, float angle_increment, int number_laser_rays){
  for(int i = 0; i < ranges.size(); i++){
    if(isValidPoint(ranges.at(i),range_max, range_min)){
      float current_angle = angle_min+i*angle_increment;
      float x = ranges.at(i)*cos(current_angle);
      float y = ranges.at(i)*sin(current_angle);
      out.push_back(Point(x,y));
    }
  }
}

bool isValidPoint(float range, float range_max, float range_min){
  return range < range_max && range > range_min;
}

void updatePCL(){

}

void updateFlappyPos(){

}


extern std::vector<Point> pcl;

int main(int argc, char **argv)
{
  ros::init(argc,argv,"flappy_automation_code");
  initNode();

  


  // Ros spin to prevent program from exiting
  ros::spin();
  return 0;
}
