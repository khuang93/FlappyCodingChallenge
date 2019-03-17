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

/**
 * @brief simpe struct for 2D point 
 * 
 */
struct Point
{
    float x; //in body coordinate, distance from flappy
    float y; //in absolute coordinate

    /**
     * @brief Construct a new Point object using x y
     * 
     * @param _x 
     * @param _y 
     */
    Point(float _x, float _y) : x(_x), y(_y)
    {
    }
    /**
     * @brief Construct a new Point object with (0, 0)
     * 
     */
    Point()
    {
        x = 0.0f;
        y = 0.0f;
    }
};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXY;

/**
 * @brief created a class so there can "global shared variables"
 * 
 */
class SubscribeAndPublish
{
  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr mypcl;      //the whole map as point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentpcl; //only points in front of the flappy
    Point flappyPos; //current flappy position
    Point flappyPos_prev; //flappy position in previos time step
    const int FPS = 30; //frame per sec, fixed in this game
    Point midPoint = Point(0.0, 0.0);
    //related to mid point of gap calculation
    double midY = 0.0;
    double midX = 0.0;
    double prev_midY = 0.0;
    double midY_unfiltered = 0.0;
    int midY_consistent = 0;

    //currently not used
    Point closestPointTop = Point(100.0, 100.0);
    Point closestPointBot = Point(100.0, 100.0);

    //vx vy in previos step
    float prev_vx = 0.0;
    float prev_vy = 0.0;

    float distX = 0.0;
    float distY = 0.0;

    //time counter to remove too old points from mypcl
    int counter = 0;

    pcl::PointXYZ min_bound;
    pcl::PointXYZ max_bound;

    //used for output
    std::ofstream midF;
    std::ofstream posF;

    Point prev_error;

  public:
    /**
     * @brief Construct a new Subscribe And Publish object
     * 
     */
    SubscribeAndPublish() : flappyPos(Point(0.0f, 0.0f)), flappyPos_prev(Point(0.0f, 0.0f)), mypcl(new pcl::PointCloud<pcl::PointXYZ>), currentpcl(new pcl::PointCloud<pcl::PointXYZ>)
    {

        //Initialization of nodehandle
        nh_ = new ros::NodeHandle();
        //Init publishers and subscribers
        pub_acc_cmd = nh_->advertise<geometry_msgs::Vector3>("/flappy_acc", 1);
        sub_vel = nh_->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &SubscribeAndPublish::velCallback, this);
        sub_laser_scan = nh_->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &SubscribeAndPublish::laserScanCallback, this);

        midF.open("midPoint.csv");
        posF.open("position.csv");

        prev_error.x = 0.0f;
        prev_error.y = 0.0f;
    }

    void velCallback(const geometry_msgs::Vector3::ConstPtr &msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    /**
     * @brief calculate the Closest Points to the current bird. Not used yet, could be used to improve robustness
     * 
     * @param currentpcl 
     * @param flappyPos 
     */
    void calculateClosestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &currentpcl, Point &flappyPos);

    /**
     * @brief update position of flappy using incoming velocity
     * 
     * @param flappyPos prev position
     * @param vx 
     * @param vy 
     */
    void updateFlappyPos(Point &flappyPos, float vx, float vy);

    /**
     * @brief add the incoming laser scan data to point cloud
     * 
     * @param mypcl 
     * @param currentpcl 
     * @param ranges input laser scan
     * @param range_max max valid range
     * @param range_min min valid range
     * @param angle_min starting angle of laser
     * @param angle_max finishing angle of laser
     * @param angle_increment angle between 2 lasers
     * @param number_laser_rays 
     * @param flappyPos current position of flappy
     */
    void convertLaserScan2PCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl, std::vector<float> ranges, float range_max, float range_min, float angle_min, float angle_max, float angle_increment, int number_laser_rays, const Point &flappyPos);

    /**
     * @brief get the middle
     * 
     * @param currentpcl 
     * @return Point 
     */
    Point calculateMidpointOfGap(PointCloudXY::Ptr &currentpcl);

  private:
    //Ros nodehandle
    ros::NodeHandle *nh_ = NULL;
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

}; //End of class SubscribeAndPublish

void initNode();

/**
 * @brief check if the point returned by the laser scan is valid (in the valid range of the laser)
 * 
 * @param range input distance
 * @param range_max max valid distance
 * @param range_min min valid distance
 * @return true means range_min < range < range_max
 * @return false else
 */
bool isValidPoint(float range, float range_max, float range_min);

/**
 * @brief filter the whole point cloud to only the points in the front of the bird, which is the next obstancle to be avoided
 * 
 * @param mypcl input, whole point cloud
 * @param currentpcl output, the point cloud currently of interest
 * @param vx velocity of bird in x
 * @param vy velocity of bird in y
 * @param flappyPosX position of bird in x coord
 */
void filterPCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl, float vx, float vy, float flappyPosX);

/**
 * @brief write point clouds as .ply file, useful while debugging.
 * 
 * @param mypcl poiter to the pcl which is to be saved
 */
void savePCL2PLY(PointCloudXY::Ptr mypcl);

/**
 * @brief compare function for pcl points by y coord, used in sorting
 * 
 * @param i point 1
 * @param j point 2
 * @return true i.y < j.y
 * @return false  i.y !< j.y
 */
bool comparePts(pcl::PointXYZ i, pcl::PointXYZ j) { return (i.y < j.y); }

double getMinXObstacleDist(pcl::PointCloud<pcl::PointXYZ>::Ptr &currentpcl, Point &flappyPos);
double getMinYObstacleDist(pcl::PointCloud<pcl::PointXYZ>::Ptr &currentpcl, Point &flappyPos);

#endif
