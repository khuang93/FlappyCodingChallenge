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

#include<fstream>



void initNode()
{
  
}

void SubscribeAndPublish::velCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  // msg has the format of geometry_msgs::Vector3
  // Example of publishing acceleration command on velocity velCallback
  geometry_msgs::Vector3 acc_cmd;


  flappyPos_prev=flappyPos;

  updateFlappyPos(flappyPos, msg->x, msg->y);
  prev_vx=msg->x;
  prev_vy=msg->y;
  filterPCL(mypcl, currentpcl, msg->x, msg->y,flappyPos.x);


  pcl::getMinMax3D(*currentpcl,min_bound, max_bound);

   float distX = min_bound.x - flappyPos.x;
   float distY = midY - flappyPos.y;
    // distX = min_bound.x - flappyPos.x;
    if(distX<0) distX = -0.05;


    
    posF<<flappyPos.x<<", "<<flappyPos.y<<std::endl;
  midF<<midX<<", "<<midY<<std::endl;

  ROS_INFO("flappyPos x: %f, y: %f, distX: %f", flappyPos.x,  flappyPos.y, distX);
  ROS_INFO("flappyVel vx: %f, vy: %f", msg->x, msg->y);
    

  pcl::io::savePLYFileASCII("pointCloud.ply", *mypcl);
  // midY = 0.5;
 

  float vx_desired = 0.2 + distX; //0.4; //0.5*std::sqrt(minDistX*minDistX+minDistY*minDistY);

  float vy_desired = distY/distX*vx_desired; //(midY-flappyPos.y); //change to distY / distX
//   if(distX>0.1) vy_desired=vy_desired/distX*0.5;
if(distX < 0) vy_desired = 0;


  Point vel = Point(vx_desired,vy_desired);
  
  // if(possibleCollision(currentpcl,flappyPos, vel)){
  //   if(minDistX > 0 && minDistX < 1 && std::abs(minDistY) < 0.2){
  //     vx_desired =  std::max(0.3*std::abs(minDistX),0.1);
  //     if(minDistY>0) {
  //       vy_desired = 4*(0.2 - minDistY);
  //     }else{
  //       vy_desired = 4*(-0.2 + minDistY);
  //     }
  //   }
  // }
  
  

ROS_INFO("flappyVelDesired vx: %f, vy: %f", vx_desired, vy_desired);
  float kp = 1.2;
  float ki = 0.8;
  acc_cmd.x = kp*(vx_desired-msg->x);
  acc_cmd.y =  kp*(vy_desired-msg->y);
  // acc_cmd.y *= std::max(0.05, msg->y);
  
  // acc_cmd.y  = 0;

  ROS_INFO("Accel ax: %f, ay: %f", acc_cmd.x,  acc_cmd.y);

  pub_acc_cmd.publish(acc_cmd);
}

void SubscribeAndPublish::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //msg has the format of sensor_msgs::LaserScan
  //print laser angle and range
 
  //ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);
  // ROS_INFO("Time Laser x: %i", msg->header.stamp.nsec);
  // ROS_INFO("Time Laser x: %i", msg->header.stamp.sec);

  int number_laser_rays = (msg->angle_max-msg->angle_min)/msg->angle_increment + 1;
  
  convertLaserScan2PCL(mypcl, currentpcl, msg->ranges, msg->range_max-0.05, msg->range_min, (float)msg->angle_min, (float)msg->angle_max, (float)msg->angle_increment, number_laser_rays, flappyPos_prev);

  this-> midPoint = getMiddleOfGap(currentpcl);
  this->midX=midPoint.x;
  this->midY=midPoint.y;

 




  getClosestPoints(currentpcl, flappyPos);
  ROS_INFO("\nMidX  %f, MidY  %f",midX,  midY);
  //  for(int i = 0; i < number_laser_rays; i++){
  //   ROS_INFO("Laser range: %f, angle: %f", msg->ranges[i], msg->angle_min+msg->angle_increment*i);
  // }
}



void SubscribeAndPublish::posCallback(const geometry_msgs::Vector3::ConstPtr& msg){

}

void SubscribeAndPublish::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

}
void SubscribeAndPublish::convertLaserScan2PCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl, std::vector<float> ranges, float range_max, float range_min, float angle_min, float angle_max, float angle_increment, int number_laser_rays, const Point& flappyPos){
    counter++;

  for(int i = 0; i < ranges.size(); i++){
    if(isValidPoint(ranges.at(i),range_max, range_min)){
      float current_angle = angle_min+i*angle_increment;
      float x = ranges.at(i)*cos(current_angle);
      float y = ranges.at(i)*sin(current_angle);
      // if(x<1 && y<1){
          if(counter>150){
              mypcl->erase(mypcl->begin());
          }
        mypcl->push_back(pcl::PointXYZ(x+flappyPos.x,y+flappyPos.y,0));
        // currentpcl->push_back(pcl::PointXYZ(x,y,0));
      // }
    }
  }
  std::sort(currentpcl->begin(),currentpcl->end(), comparePts );

  pcl::io::savePLYFileASCII("currentpcl.ply", *currentpcl);
}

bool isValidPoint(float range, float range_max, float range_min){
  return range < range_max && range > range_min;
}

void filterPCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl, float vx, float vy, float flappyPosX){

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  if(mypcl->size()==0) return;
  for (int i = 0; i < (*mypcl).size(); i++)
  {
    if ((mypcl->points[i].x- flappyPosX) < -0.35f || (mypcl->points[i].x- flappyPosX) >1.2f ){
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(mypcl);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*currentpcl);

}

void SubscribeAndPublish::updateFlappyPos(Point& flappyPos, float vx, float vy){
//   vx=0.5*(vx+prev_vx);
//   vy=0.5*(vy+prev_vy);  
//  vx=prev_vx;
//   vy=prev_vy;  
  float dX = vx/30;
  float dY = vy/30;

  flappyPos.x+=dX;
  flappyPos.y+=dY;
}

Point SubscribeAndPublish::getMiddleOfGap(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl){
  double midY = this->midY;
  double midX = this->midX;
  double maxGap = 0.0;
  if(currentpcl->size()==0) return Point(0.0,0.0);
  for(int i = 0; i < currentpcl->size()-1;i++){
    double gap = currentpcl->at(i+1).y-currentpcl->at(i).y;
    if(gap>maxGap){
      maxGap=gap;
      double midY_tmp = 0.5*(currentpcl->at(i+1).y+currentpcl->at(i).y);
      if(midY_tmp<2 && midY_tmp>-1.2){
        midY = midY_tmp;
        double midX = 0.5*(currentpcl->at(i+1).x+currentpcl->at(i).x);
      }
    }
  } 
  if(maxGap < 0.1){ //was 0.1
    midY = this->midY; //was 0
    // if(flappyPos.y>0 && min_bound.y > -1.2) midY=min_bound.y+0.1;
    // else if(max_bound.y < 1.5)  midY=max_bound.y-0.1;
    // if(2.5 < midY || midY<-1.4) midY = 0.2;
  }


  midX = 0.5*(max_bound.x+min_bound.x);
  return Point(midX,midY);
}

void SubscribeAndPublish::getClosestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, Point& flappyPos){
  float minDistTop = 100.0f;
  float minDistBot = 100.0f;
  if(currentpcl->size()==0) return;

  for(int i = 0; i < currentpcl->size();i++){
    float distY = flappyPos.y - currentpcl->at(i).y;
    float distX = flappyPos.x - currentpcl->at(i).x;
    float dist = sqrt(distX*distX+distY*distY);
    if(distY<0){
      if(dist<minDistTop && dist < 1){
            minDistTop=dist;
            this->closestPointTop.x = distX;
            this->closestPointTop.y = distY;
      }
    }else{
       if(dist<minDistBot && dist < 1){
            minDistBot=dist;
            this->closestPointBot.x = distX;
            this->closestPointBot.y = distY;
      }
    }
  } 
}

double getMinXObstacleDist(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, Point& flappyPos){
  double minDist = 100.0;
  double minDistAbs = 100.0;
  for(int i = 0; i < currentpcl->size();i++){
    double distX = flappyPos.x - currentpcl->at(i).x;
    double absDist = std::abs(distX);
    if(absDist<minDistAbs){
      minDistAbs = absDist;
      minDist=distX;
    }
  } 
  return minDist;
}

double getMinYObstacleDist(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, Point& flappyPos){
  double minDist = 100.0;
  double minDistAbs = 100.0;
  for(int i = 0; i < currentpcl->size();i++){
    double distY = flappyPos.y - currentpcl->at(i).y;
    double absDist = std::abs(distY);
    if(absDist<minDistAbs){
      minDistAbs = absDist;
      minDist=distY;
    }
  } 
  return minDist;
}

bool possibleCollision(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, Point& flappyPos, Point& vel){
  Point newPos = Point(flappyPos.x+vel.x,flappyPos.y+vel.y);
  float th = 0.2;
  for(int i = 0; i < currentpcl->size();i++){
    float distX = newPos.x - currentpcl->at(i).x;
    float distY = newPos.y - currentpcl->at(i).y;
    if(distX < th || distY<th) return true;
  }
  

  return false;
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"flappy_automation_code");
  initNode();
  SubscribeAndPublish SAPObject;
  

  // Ros spin to prevent program from exiting
  ros::spin();
  return 0;
}
