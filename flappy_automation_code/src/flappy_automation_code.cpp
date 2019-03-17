/**
 * @file flappy_automation_code.cpp
 * @author your name (you@domain.com) modified by Kailin Huang, kailin-huang@outlook.com
 * @brief code for Flyability Application
 * @version 0.3
 * @date 2019-03-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "ros/ros.h"
#include "flappy_automation_code/flappy_automation_code.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include <vector>
#include <math.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <algorithm>
#include <functional>
#include <array>
#include <fstream>
#include <math.h>

//global variables in this file for easier tuning
static const int max_time_counter = 160; //points older than this will be removed from the point cloud
static const float vx_base = 0.32f;
//filter for the current interesting points in x
static const float filter_x_min = -0.18f; //was -0.35
static const float filter_x_max = 1.7f;   //was 1.8
//filter out walls
static const float filter_y_max = 2.1f;  //was 1.8
static const float filter_y_min = -1.1f; //was -1.3

static const float min_gap_TH = 0.15f; //was 0.1

//PID values
//for x
static const float kp_x = 0.9;
//for y
static const float kp = 1.1;      //1.1
static const float kp_vy = 1.13;  //1.1
static const float ki = 0.6 * 30; //0.5
static const float kd = 0.85;     //0.85

void initNode()
{
}

void SubscribeAndPublish::velCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  // msg has the format of geometry_msgs::Vector3
  // Example of publishing acceleration command on velocity velCallback
  geometry_msgs::Vector3 acc_cmd;

  this->flappyPos_prev = this->flappyPos;

  updateFlappyPos(this->flappyPos, msg->x, msg->y);
  this->prev_vx = msg->x;
  this->prev_vy = msg->y;

  filterPCL(this->mypcl, this->currentpcl, msg->x, msg->y, flappyPos.x);

  //get boundary of currentpcl
  pcl::getMinMax3D(*currentpcl, min_bound, max_bound);

  //distance vector from flappy to the middle of the gap
  distX = 0;
  distY = 0;

  if (!currentpcl->empty())
  {
    distX = min_bound.x - flappyPos.x - 0.02;

    distY = midY - flappyPos.y;
  }

  if (distX < 0)
    distX = distX > -0.05 ? distX : -0.05; //was -0.05

  posF << flappyPos.x << ", " << flappyPos.y << std::endl;
  midF << midX << ", " << midY << std::endl;

  ROS_INFO("flappyPos x: %f, y: %f, distX: %f", flappyPos.x, flappyPos.y, distX);
  ROS_INFO("flappyVel vx: %f, vy: %f", msg->x, msg->y);

  // pcl::io::savePLYFileASCII("pointCloud.ply", *mypcl);

  float vx_desired = vx_base + 0.5 * distX;

  float vy_desired = distY; // /distX*vx_desired;

  //if in the middle of a gap, dont move
  if (distX < -0.04) //was -0.025
  {
    vy_desired = 0;
    distY = 0;
  }
  ROS_INFO("flappyVelDesired vx_desired: %f, vy_desired: %f", vx_desired, vy_desired);
  double integral_x = distX + prev_error.x;
  double integral_y = distY + prev_error.y;

  double dT = 1.0 / this->FPS;
  double diff_x = distX - prev_error.x;
  double diff_y = distY - prev_error.y;
  prev_error.x = distX;
  prev_error.y = distY;

  //TODO take into account the closest point in Y direction!!! (Maybe split the PCL into upper and lower)

  // acc_cmd.x = ki*distX + kp*(vx_desired-msg->x)*dT + kd*diff_x*30;
  //P Control of vx
  acc_cmd.x = kp_x * (vx_desired - msg->x);

  //acc_cmd.y = kp*distY + ki*(vy_desired-msg->y)*dT + kd*diff_y*30;

  //PID of pos y with additional term for vy
  acc_cmd.y = kp * distY + kp_vy * (vy_desired - msg->y) + ki * integral_y * dT + kd * diff_y * this->FPS;

  ROS_INFO("Accel ax: %f, ay: %f", acc_cmd.x, acc_cmd.y);

  pub_acc_cmd.publish(acc_cmd);
}

void SubscribeAndPublish::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  //msg has the format of sensor_msgs::LaserScan
  //print laser angle and range for debug
  /*
  ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);
  ROS_INFO("Time Laser x: %i", msg->header.stamp.nsec);
  ROS_INFO("Time Laser x: %i", msg->header.stamp.sec);
  */
  int number_laser_rays = (msg->angle_max - msg->angle_min) / msg->angle_increment + 1;

  convertLaserScan2PCL(mypcl, currentpcl, msg->ranges, msg->range_max - 0.05, msg->range_min, (float)msg->angle_min, (float)msg->angle_max, (float)msg->angle_increment, number_laser_rays, flappyPos_prev);

  this->midPoint = calculateMidpointOfGap(currentpcl);
  this->midX = midPoint.x;
  this->midY = midPoint.y;

  // calculateClosestPoints(currentpcl, flappyPos); //currently not using this

  //debug output
  ROS_INFO("MidX  %f, MidY  %f", midX, midY);

  //debug outputs
  /*
  for(int i = 0; i < number_laser_rays; i++){
    ROS_INFO("Laser range: %f, angle: %f", msg->ranges[i], msg->angle_min+msg->angle_increment*i);
  }
  */
}

void SubscribeAndPublish::convertLaserScan2PCL(
    PointCloudXY::Ptr mypcl,
    PointCloudXY::Ptr currentpcl,
    std::vector<float> ranges,
    float range_max,
    float range_min,
    float angle_min,
    float angle_max,
    float angle_increment,
    int number_laser_rays,
    const Point &flappyPos)
{
  counter++;

  for (int i = 0; i < ranges.size(); i++)
  {
    if (isValidPoint(ranges.at(i), range_max, range_min))
    {
      float current_angle = angle_min + i * angle_increment;
      float x = ranges.at(i) * cos(current_angle); // - prev_vx/30*0.5;
      float y = ranges.at(i) * sin(current_angle); //  - prev_vy/30*0.5;

      if (counter > max_time_counter)
      {
        mypcl->erase(mypcl->begin());
      }
      mypcl->push_back(pcl::PointXYZ(x + flappyPos_prev.x, y + flappyPos_prev.y, 0));
    }
  }
  std::sort(currentpcl->begin(), currentpcl->end(), comparePts);

  pcl::io::savePLYFileASCII("pointCloud.ply", *currentpcl);
}

void SubscribeAndPublish::updateFlappyPos(Point &flappyPos, float vx, float vy)
{
  float dX = vx / this->FPS;
  float dY = vy / this->FPS;

  flappyPos.x += dX;
  flappyPos.y += dY;
}

Point SubscribeAndPublish::calculateMidpointOfGap(pcl::PointCloud<pcl::PointXYZ>::Ptr &currentpcl)
{
  double _midY = this->midY;
  this->prev_midY = this->midY;
  // double _midX  = this->midX;
  double maxGap = 0.0;
  //find the larges gap in y direction along the point cloud, which is already sorted
  if (currentpcl->size() == 0)
    return Point(0.0, 0.0);
  for (int i = 0; i < currentpcl->size() - 1; i++)
  {
    double gap = currentpcl->at(i + 1).y - currentpcl->at(i).y;
    if (gap > maxGap)
    {
      maxGap = gap;
      double midY_tmp = 0.5 * (currentpcl->at(i + 1).y + currentpcl->at(i).y);
      // _midX =  0.5 * (currentpcl->at(i + 1).x + currentpcl->at(i).x);
      if (midY_tmp < 2.2 && midY_tmp > -1.2)
      {
        _midY = midY_tmp;
      }
    }
  }

  pcl::getMinMax3D(*currentpcl, min_bound, max_bound);
  double _midX = 0.5 * (max_bound.x + min_bound.x);

  //if gap is far far above or below and not captured by lidar
  if (maxGap < min_gap_TH) //max gap size
  {                        //was 0.1
    //0.5 is the midpoint of of the y range [-1.5, 2.5]
    if (this->flappyPos.y < 0.5)
    {
      _midY = flappyPos.y + 0.3;
      return Point(_midX, _midY);
    }
    else
    {
      _midY = flappyPos.y - 0.3;
      return Point(_midX, _midY);
    }
    _midY = prev_midY;
  }

  float TH = 0.05f;
  double delta_midY = _midY - this->midY_unfiltered;
  double delta_midY_abs = std::abs(delta_midY);
  if (delta_midY_abs < TH)
  {
    midY_consistent++;
  }
  else
  {
    midY_consistent = 0;
  }
  this->midY_unfiltered = _midY;
  //degug output
  //ROS_INFO("#################################### prev_midY %f, _midY %f, midY_consistent  %i", prev_midY, _midY, midY_consistent);

  //if midY change is large or bird very close to the obstacles, require consistency before change midY
  if (delta_midY_abs > 0.75 /* || distX < 0.2*/)
  {
    if (midY_consistent > 12)
    {
      prev_midY = _midY;

      //compensation for large midY changes to avoid "scratching the stones"
      if (delta_midY_abs > 0.75)
      {
        if (delta_midY > 0)
          _midY -= 0.03;
        else
          _midY += 0.03;
      }
    }
    else
    {
      _midY = prev_midY;
    }
  }
  else
  {
    prev_midY = _midY;
  }

  this->midY = _midY;
  return Point(midX, _midY);
}

void SubscribeAndPublish::calculateClosestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &currentpcl, Point &flappyPos)
{
  float minDistTop = 100.0f;
  float minDistBot = 100.0f;
  if (currentpcl->size() == 0)
    return;

  for (int i = 0; i < currentpcl->size(); i++)
  {
    //float distY = 0, distX = 0;
    if (!currentpcl->empty())
    {
      distY = flappyPos.y - currentpcl->at(i).y;
      distX = flappyPos.x - currentpcl->at(i).x;
    }

    float dist = sqrt(distX * distX + distY * distY);
    if (distY < 0)
    {
      if (dist < minDistTop && dist < 1)
      {
        minDistTop = dist;
        this->closestPointTop.x = distX;
        this->closestPointTop.y = distY;
      }
    }
    else
    {
      if (dist < minDistBot && dist < 1)
      {
        minDistBot = dist;
        this->closestPointBot.x = distX;
        this->closestPointBot.y = distY;
      }
    }
  }
}

bool isValidPoint(float range, float range_max, float range_min)
{
  return range < range_max && range > range_min;
}

void filterPCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl, float vx, float vy, float flappyPosX)
{

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  if (mypcl->size() == 0)
    return;
  for (int i = 0; i < (*mypcl).size(); i++)
  {
    //filter out only the points in certain range
    if ((mypcl->points[i].x - flappyPosX) > filter_x_min && (mypcl->points[i].x - flappyPosX) < filter_x_max && mypcl->points[i].y < filter_y_max && mypcl->points[i].y > filter_y_min)
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(mypcl);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*currentpcl);
}

//not used
double getMinXObstacleDist(pcl::PointCloud<pcl::PointXYZ>::Ptr &currentpcl, Point &flappyPos)
{
  double minDist = 100.0;
  double minDistAbs = 100.0;
  for (int i = 0; i < currentpcl->size(); i++)
  {
    double distX = flappyPos.x - currentpcl->at(i).x;
    double absDist = std::abs(distX);
    if (absDist < minDistAbs)
    {
      minDistAbs = absDist;
      minDist = distX;
    }
  }
  return minDist;
}

//not used
double getMinYObstacleDist(pcl::PointCloud<pcl::PointXYZ>::Ptr &currentpcl, Point &flappyPos)
{
  double minDist = 100.0;
  double minDistAbs = 100.0;
  for (int i = 0; i < currentpcl->size(); i++)
  {
    double distY = flappyPos.y - currentpcl->at(i).y;
    double absDist = std::abs(distY);
    if (absDist < minDistAbs)
    {
      minDistAbs = absDist;
      minDist = distY;
    }
  }
  return minDist;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flappy_automation_code");
  initNode();
  SubscribeAndPublish SAPObject;

  // Ros spin to prevent program from exiting
  ros::spin();
  return 0;
}
