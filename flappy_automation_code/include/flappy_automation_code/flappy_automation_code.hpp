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
#include "gap.hpp"

// struct Point{
//   float x; //in body coordinate, distance from flappy
//   float y; //in absolute coordinate
//   int type; //0 is obstacle, 1 is wall
//   Point(float _x, float _y):x(_x),y(_y){ 
//       type = 0;
//   }
// };

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXY;


class SubscribeAndPublish
{
public: //"Global" constants
    const float PIPE_WIDTH = 0.5;
    const float MIN_GAP_SIZE = 0.3;

private: //Ros stuffs
    //Ros nodehandle
    ros::NodeHandle* nh_= NULL;
    //Publisher for acceleration command
    ros::Publisher pub_acc_cmd;
    //Subscriber for velocity
    ros::Subscriber sub_vel;
    //Subscriber for laser scan
    ros::Subscriber sub_laser_scan;

private: //shared stuffs
    float y_top_wall = 0.0;
    float y_bottom_wall = 0.0; 
    int framesElapsed = 0;
    int state_bird = 0; //0 in free space, 1 in pipe
    pcl::PointCloud<pcl::PointXYZ>::Ptr mypcl; //the whole map as point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentpcl; //only points in front of the flappy
    pcl::PointXYZ flappyPos;
    pcl::PointXYZ flappyPos_prev;
    pcl::PointXYZ vel_prev= pcl::PointXYZ(0.0f, 0.0f, 0.0f);
    const int FPS = 30; //frame per sec, fixed in this game
    pcl::PointXYZ midPoint = pcl::PointXYZ(0.0f, 0.0f, 0.0f);
    pcl::PointXYZ midPoint_old = pcl::PointXYZ(0.0f, 0.0f, 0.0f);
    double midY=0.0;
    double midY_old=0.0;
    double midX=0.0;
    pcl::PointXYZ closestPointTop  = pcl::PointXYZ(100.0f, 100.0f, 0.0f);
    pcl::PointXYZ closestPointBot  = pcl::PointXYZ(100.0f, 100.0f, 0.0f);
    std::vector<Gap> Gaps;


public:
    //Constructor
    SubscribeAndPublish():flappyPos(pcl::PointXYZ(0.0f,0.0f,0.0f)),flappyPos_prev(pcl::PointXYZ(0.0f,0.0f,0.0f)),mypcl(new pcl::PointCloud<pcl::PointXYZ>),currentpcl(new pcl::PointCloud<pcl::PointXYZ>)
    {
        //Initialization of nodehandle
        nh_ = new ros::NodeHandle();
        //Init publishers and subscribers
        pub_acc_cmd = nh_->advertise<geometry_msgs::Vector3>("/flappy_acc",1);
        sub_vel = nh_->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &SubscribeAndPublish::velCallback, this);
        sub_laser_scan = nh_->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &SubscribeAndPublish::laserScanCallback, this);

    }
    //Callbacks
    void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    //Custom functions in class

    void getClosestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, pcl::PointXYZ& flappyPos);

    void getMiddleOfGap(PointCloudXY::Ptr& currentpcl);

    void calculateGaps(PointCloudXY::Ptr& currentpcl);

    void assignWeight2Gaps();
    
    void updateFlappyPos(pcl::PointXYZ& flappyPos, float vx, float vy);

    float getXOfNextPipe();
    
};//End of class SubscribeAndPublish



void initNode();

//Custom Fucntions out of class

void convertLaserScan2PCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl,
    std::vector<float> ranges, float range_max, float range_min, float angle_min, float angle_max, 
    float angle_increment, int number_laser_rays,  const pcl::PointXYZ&flappyPos);

bool isValidPoint(float range, float range_max, float range_min);
void filterPCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl, float vx, float vy, float flappyPosX);

void savePCL2PLY(PointCloudXY::Ptr mypcl);

bool comparePts (pcl::PointXYZ i, pcl::PointXYZ j) { return (i.y<j.y); }


pcl::PointXYZ getClosestPointBot(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, pcl::PointXYZ& flappyPos);
double getMinXObstacleDist(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, pcl::PointXYZ& flappyPos);
double getMinYObstacleDist(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, pcl::PointXYZ& flappyPos);

float calculatePointDistance(pcl::PointXYZ p1, pcl::PointXYZ p2);

int countPointsInXRange(float min, float max, pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl);
int countPointsInYRange(float min, float max, pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl);
int countPointsInRadius(pcl::PointXYZ pt, float r, pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl);

int filterPointsInXRange(float min, float max, pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl);
int filterPointsInYRange(float min, float max, pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl);

#endif
