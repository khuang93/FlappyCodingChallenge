#ifndef FLAPPY_AUTOMATION_CODE_H_
#define FLAPPY_AUTOMATION_CODE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

struct Point{
  float x; //in body coordinate, distance from flappy
  float y; //in absolute coordinate
  int type; //0 is obstacle, 1 is wall
  Point(float _x, float _y):x(_x),y(_y){ 
      type = 0;
  }
};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXY;


class SubscribeAndPublish
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr mypcl; //the whole map as point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentpcl; //only points in front of the flappy
    Point flappyPos;
    Point flappyPos_prev;
    const int FPS = 30; //frame per sec, fixed in this game
    Point midPoint = Point(0.0,0.0);
    double midY=0.0;
    double midX=0.0;
    Point closestPointTop  = Point(100.0,100.0);
    Point closestPointBot  = Point(100.0,100.0);
    float prev_vx=0.0;
    float prev_vy=0.0;
    int counter = 0;
      pcl::PointXYZ min_bound;
  pcl::PointXYZ max_bound;

public:
    //Constructor
    SubscribeAndPublish():flappyPos(Point(0.0f,0.0f)),flappyPos_prev(Point(0.0f,0.0f)),mypcl(new pcl::PointCloud<pcl::PointXYZ>),currentpcl(new pcl::PointCloud<pcl::PointXYZ>)
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
        //subscribers
        sub_flappy_pos = nh_->subscribe<geometry_msgs::Vector3>("/flappy_pos",1, &SubscribeAndPublish::posCallback, this);
        sub_pcl = nh_->subscribe<sensor_msgs::PointCloud2>("/PCL",1, &SubscribeAndPublish::pclCallback, this);



    }

    void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void posCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void getClosestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, Point& flappyPos);

    void updateFlappyPos(Point& flappyPos, float vx, float vy);
    void convertLaserScan2PCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl,std::vector<float> ranges, float range_max, float range_min, float angle_min, float angle_max, float angle_increment, int number_laser_rays,  const Point&flappyPos);
    Point getMiddleOfGap(PointCloudXY::Ptr& currentpcl);
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





void initNode();



bool isValidPoint(float range, float range_max, float range_min);
void filterPCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl, float vx, float vy, float flappyPosX);
// void updateFlappyPos(Point& flappyPos, float vx, float vy);
void savePCL2PLY(PointCloudXY::Ptr mypcl);

// bool comparePts (Point i,Point j) { return (i.y<j.y); }
bool comparePts (pcl::PointXYZ i, pcl::PointXYZ j) { return (i.y<j.y); }


Point getClosestPointBot(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, Point& flappyPos);
double getMinXObstacleDist(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, Point& flappyPos);
double getMinYObstacleDist(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, Point& flappyPos);
bool possibleCollision(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, Point& flappyPos, Point& vel);

#endif
