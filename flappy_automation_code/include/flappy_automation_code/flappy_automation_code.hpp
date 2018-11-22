#ifndef FLAPPY_AUTOMATION_CODE_H_
#define FLAPPY_AUTOMATION_CODE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Initialization of nodehandle
        nh_ = new ros::NodeHandle();
        //Init publishers and subscribers
        pub_acc_cmd = nh_->advertise<geometry_msgs::Vector3>("/flappy_acc",1);
        sub_vel = nh_->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &SubscribeAndPublish::velCallback, this);
        sub_laser_scan = nh_->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &SubscribeAndPublish::laserScanCallback, this);

        //additional state publisher
        pub_flappy_pos = nh_->advertise<geometry_msgs::Vector3>("/flappy_pos",1);
        pub_pcl = nh_->advertise<sensor_msgs::PointCloud2>("/PCL",1);

        sub_flappy_pos = nh_->subscribe<geometry_msgs::Vector3>("/flappy_pos",1, &SubscribeAndPublish::posCallback, this);
        sub_pcl = nh_->subscribe<sensor_msgs::PointCloud2>("/PCL",1, &SubscribeAndPublish::pclCallback, this);
    }

/*     void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
    {
       // PUBLISHED_MESSAGE_TYPE output;
        //.... do something with the input and generate the output...
        //pub_.publish(output);  

    } */
    void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void posCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  
private:
    //Ros nodehandle
    ros::NodeHandle* nh_= NULL;
    //Publisher for acceleration command
    ros::Publisher pub_acc_cmd;
    //Subscriber for velocity
    ros::Subscriber sub_vel;
    //Subscriber for laser scan
    ros::Subscriber sub_laser_scan;

    //publisher flappy position
    ros::Publisher pub_flappy_pos;
    //publisher global pcl
    ros::Publisher pub_pcl;
    //Subscriber flappy position
    ros::Subscriber sub_flappy_pos;
    //Subscriber global pcl
    ros::Subscriber sub_pcl;

};//End of class SubscribeAndPublish



struct Point{
  float x; //in body coordinate, distance from flappy
  float y; //in absolute coordinate
  int type; //0 is obstacle, 1 is wall
  Point(float _x, float _y):x(_x),y(_y){ 
      type = 0;
  }
};

typedef pcl::PointCloud<Point> PointCloudXY;

void initNode();


void convertLaserScan2PCL(std::vector<Point>& out, std::vector<float> ranges, float range_max, float range_min, float angle_min, float angle_max, float angle_increment, int number_laser_rays);
bool isValidPoint(float range, float range_max, float range_min);
void updatePCL(geometry_msgs::Vector3::ConstPtr& msg);
void updateFlappyPos();

#endif
