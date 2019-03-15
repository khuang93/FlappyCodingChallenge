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
  float distX =0, distY=0;
 if(!currentpcl->empty()){
 
    distX = min_bound.x - flappyPos.x -0.02;
    distY = midY - flappyPos.y;
 }

    // distX = min_bound.x - flappyPos.x;
    if(distX<0) distX =  distX > -0.05 ? distX : -0.05;

  
    
  posF<<flappyPos.x<<", "<<flappyPos.y<<std::endl;
  midF<<midX<<", "<<midY<<std::endl;


  ROS_INFO("flappyPos x: %f, y: %f, distX: %f", flappyPos.x,  flappyPos.y, distX);
  ROS_INFO("flappyVel vx: %f, vy: %f", msg->x, msg->y);
    

  pcl::io::savePLYFileASCII("pointCloud.ply", *mypcl);
  // midY = 0.5;
 

  float vx_desired = 0.27 + 0.5*distX; //0.4; //0.5*std::sqrt(minDistX*minDistX+minDistY*minDistY);

  float vy_desired = distY; // /distX*vx_desired; //(midY-flappyPos.y); //change to distY / distX
//   if(distX>0.1) vy_desired=vy_desired/distX*0.5;
  
  



  float kp =1.1;
  float kp_x  = 0.9;
  float ki = 1.1;
  float kd = 0.9;
  if(distX < -0.02) {
    vy_desired = 0;
    // distY = 0;
  }
  ROS_INFO("flappyVelDesired vx: %f, vy: %f", vx_desired, vy_desired);
  double integral_x =distX + prev_error.x;
  double integral_y =distY + prev_error.y;

  double oneOverdT=30;
    double dT=1/oneOverdT;
  double diff_x = distX - prev_error.x;
  double diff_y = distY - prev_error.y;
  prev_error.x=distX;
  prev_error.y=distY;
  // diff_vy_error = vy_desired - 

//TODO take into account the closest point in Y direction!!! (Maybe split the PCL into upper and lower)


  // acc_cmd.x = ki*distX + kp*(vx_desired-msg->x)*dT + kd*diff_x*30;
  acc_cmd.x = kp_x*(vx_desired-msg->x);

  //acc_cmd.y = kp*distY + ki*(vy_desired-msg->y)*dT + kd*diff_y*30;


  acc_cmd.y = kp*distY + ki*(vy_desired-msg->y) + kd*diff_y*oneOverdT;
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
      float x = ranges.at(i)*cos(current_angle);// - prev_vx/30*0.5;
      float y = ranges.at(i)*sin(current_angle);// - prev_vy/30*0.5;
      // if(x<1 && y<1){
          if(counter>180){
              mypcl->erase(mypcl->begin());
          }
        mypcl->push_back(pcl::PointXYZ(x+flappyPos_prev.x,y+flappyPos_prev.y,0));
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
    if ((mypcl->points[i].x- flappyPosX) > -0.35f && (mypcl->points[i].x- flappyPosX) < 1.1f && mypcl->points[i].y < 1.8 && mypcl->points[i].y > -1.3 ){
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(mypcl);
  extract.setIndices(inliers);
  extract.setNegative(false);
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
  double _midY = this->midY;
  // double midX = this->midX;
  prev_midY=this->midY;
  double maxGap = 0.0;
  if(currentpcl->size()==0) return Point(0.0,0.0);
  for(int i = 0; i < currentpcl->size()-1;i++){
    double gap = currentpcl->at(i+1).y-currentpcl->at(i).y;
    if(gap>maxGap){
      maxGap=gap;
      double midY_tmp = 0.5*(currentpcl->at(i+1).y+currentpcl->at(i).y);
      if(midY_tmp<2.2 && midY_tmp>-1.2){
        _midY = midY_tmp;
        double midX = 0.5*(currentpcl->at(i+1).x+currentpcl->at(i).x);
      }
    }
  } 

  midX = 0.5*(max_bound.x+min_bound.x);

  //if gap is far far above or below and not captured by lidar
  if(maxGap < 0.1){ //was 0.1
//0.5 is the midpoint of the bird
    if(this->flappyPos.y<0.5) {
      _midY = flappyPos.y + 0.3;
      this->midY = _midY;
        return Point(midX,_midY);
      }
    else{
       _midY = flappyPos.y - 0.3;
       this->midY = _midY;
         return Point(midX,_midY);
       }
   _midY = prev_midY; //was 0
    // if(flappyPos.y>0 && min_bound.y > -1.2) midY=min_bound.y+0.1;
    // else if(max_bound.y < 1.5)  midY=max_bound.y-0.1;
    // if(2.5 < midY || midY<-1.4) midY = 0.2;
  }


  
  float TH = 0.05f;
  double delta_midY = std::abs(_midY-midY_unfiltered);
  if(_midY-midY_unfiltered<TH && midY_unfiltered-_midY<TH){
    midY_consistent++;
  }else{
    midY_consistent=0;
  }
midY_unfiltered=_midY;
  ROS_INFO("#################################### prev_midY %f, _midY %f, midY_consistent  %i", prev_midY, _midY, midY_consistent);

//if midY change is large require consistency
if(delta_midY>0.75){
   if(midY_consistent>12){
      prev_midY=_midY;
  }else{
      _midY=prev_midY;
  }
}else{
  prev_midY=_midY;
}
 
  this->midY = _midY;
  return Point(midX,_midY);

}

void SubscribeAndPublish::getClosestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentpcl, Point& flappyPos){
  float minDistTop = 100.0f;
  float minDistBot = 100.0f;
  if(currentpcl->size()==0) return;

  for(int i = 0; i < currentpcl->size();i++){
    float distY=0, distX=0;
    if(!currentpcl->empty()){
      distY = flappyPos.y - currentpcl->at(i).y;
      distX = flappyPos.x - currentpcl->at(i).x;
    }

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
