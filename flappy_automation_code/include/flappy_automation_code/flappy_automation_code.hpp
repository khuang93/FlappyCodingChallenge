#ifndef FLAPPY_AUTOMATION_CODE_H_
#define FLAPPY_AUTOMATION_CODE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl_conversions/pcl_conversions.h>

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
     pcl::PointCloud<pcl::PointXYZ>::Ptr mypcl;
    Point flappyPos;
    //std::vector<Point> pcl;
    const int FPS = 30;

public:
    //Constructor
    SubscribeAndPublish(ros::NodeHandle* _nh_):nh_(_nh_),flappyPos(Point(0.0f,0.0f)),mypcl(new pcl::PointCloud<pcl::PointXYZ>)
        // laser_sub_(*nh_, "base_scan", 10),
        // laser_notifier_(laser_sub_,listener_, "base_link", 10)
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


        //         laser_notifier_.registerCallback(
        // boost::bind(&SubscribeAndPublish::scanCallback, this, _1));
        // laser_notifier_.setTolerance(ros::Duration(0.01));
        // scan_pub_ = nh_->advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    }

/*  void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
    {
    PUBLISHED_MESSAGE_TYPE output;
        .... do something with the input and generate the output...
        pub_.publish(output);  

    } */

    void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void posCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // laser_geometry::LaserProjection projector_;
    // tf::TransformListener listener_;
    // message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    // tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    // ros::Publisher scan_pub_;



    // void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    // {
    //     sensor_msgs::PointCloud cloud;
    //     try
    //     {
    //         projector_.transformLaserScanToPointCloud(
    //         "base_link",*scan_in, cloud,listener_);
    //     }
    //     catch (tf::TransformException& e)
    //     {
    //         std::cout << e.what();
    //         return;
    //     }
        
    //     // Do something with cloud.

    //     scan_pub_.publish(cloud);
    // } 

  
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


// class LaserScanToPointCloud{

// public:

//   ros::NodeHandle n_;
//   laser_geometry::LaserProjection projector_;
//   tf::TransformListener listener_;
//   message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
//   tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
//   ros::Publisher scan_pub_;

//   LaserScanToPointCloud(ros::NodeHandle n) : 
//     n_(n),
//     laser_sub_(n_, "base_scan", 10),
//     laser_notifier_(laser_sub_,listener_, "base_link", 10)
//   {
//     laser_notifier_.registerCallback(
//       boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
//     laser_notifier_.setTolerance(ros::Duration(0.01));
//     scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
//   }

//   void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
//   {
//     sensor_msgs::PointCloud cloud;
//     try
//     {
//         projector_.transformLaserScanToPointCloud(
//           "base_link",*scan_in, cloud,listener_);
//     }
//     catch (tf::TransformException& e)
//     {
//         std::cout << e.what();
//         return;
//     }
    
//     // Do something with cloud.

//     scan_pub_.publish(cloud);

//   }
// };




void initNode();


void convertLaserScan2PCL(PointCloudXY::Ptr out, std::vector<float> ranges, float range_max, float range_min, float angle_min, float angle_max, float angle_increment, int number_laser_rays,  const Point&flappyPos);
bool isValidPoint(float range, float range_max, float range_min);
void filterPCL(PointCloudXY::Ptr mypcl, float vx, float vy, float flappyPosX);
void updateFlappyPos(Point& flappyPos, float vx, float vy);

#endif
