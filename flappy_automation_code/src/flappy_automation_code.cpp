#include "ros/ros.h"
#include "flappy_automation_code/flappy_automation_code.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include <vector>
#include <math.h>
#include <pcl/cloud_iterator.h>
#include <pcl/filters/passthrough.h>
#include <pcl-1.7/pcl/filters/extract_indices.h>
#include <algorithm>
#include <functional>
#include <array>




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
  filterPCL(mypcl, currentpcl, msg->x, msg->y,flappyPos.x);

  ROS_INFO("flappyPos x: %f, y: %f", flappyPos.x,  flappyPos.y);

/*   for(int i = 0; i < mypcl->size(); i++){
      //if((mypcl->at(i).x-flappyPos.x) < 0) 
      ROS_INFO("PCL x: %f, y: %f", mypcl->at(i).x-flappyPos.x,  mypcl->at(i).y);
      
  } */

/*   for(int i = 0; i < currentpcl->size(); i++){
      //if((mypcl->at(i).x-flappyPos.x) < 0)
      ROS_INFO("CurrentPCL x: %f, y: %f", currentpcl->at(i).x-flappyPos.x,  currentpcl->at(i).y);
      
  } */

  pcl::io::savePLYFileASCII("pointCloud.ply", *mypcl);
   

  acc_cmd.x = 0.1;
  acc_cmd.y = midY;
/*   if(midY<-1){
    acc_cmd.y=0.1;
  }else if(midY>-1&&midY>1){
    acc_cmd.y=0.1;
  }else{
        acc_cmd.y = midY;
  } */

  pub_acc_cmd.publish(acc_cmd);
}

void SubscribeAndPublish::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //msg has the format of sensor_msgs::LaserScan
  //print laser angle and range
 
  //ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);
  ROS_INFO("Time Laser x: %f", msg->header.stamp.nsec);
  ROS_INFO("Time Laser x: %f", msg->header.stamp.sec);

  int number_laser_rays = (msg->angle_max-msg->angle_min)/msg->angle_increment + 1;
  ROS_INFO("Laser number_laser_rays: %i", number_laser_rays);
 
  // projector_.transformLaserScanToPointCloud("base_link",*msg,cloud,listener_);
  
  convertLaserScan2PCL(mypcl, currentpcl, current_pcl, msg->ranges, msg->range_max, msg->range_min, (float)msg->angle_min, (float)msg->angle_max, (float)msg->angle_increment, number_laser_rays, flappyPos_prev);

  // this->midY = getMiddleOfGap(current_pcl);
  // ROS_INFO("MidY  %f", midY);
   for(int i = 0; i < number_laser_rays; i++){
    ROS_INFO("Laser range: %f, angle: %f", msg->ranges[i], msg->angle_min+msg->angle_increment*i);

  }



}



void SubscribeAndPublish::posCallback(const geometry_msgs::Vector3::ConstPtr& msg){

}

void SubscribeAndPublish::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

}
void convertLaserScan2PCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl,std::vector<Point>& current_pcl, std::vector<float> ranges, float range_max, float range_min, float angle_min, float angle_max, float angle_increment, int number_laser_rays, const Point& flappyPos){
  // currentpcl->clear();
  current_pcl.clear();
  for(int i = 0; i < ranges.size(); i++){
    if(isValidPoint(ranges.at(i),range_max, range_min)){
      float current_angle = angle_min+i*angle_increment;
      float x = ranges.at(i)*cos(current_angle)+flappyPos.x;
      float y = ranges.at(i)*sin(current_angle)+flappyPos.y;
      mypcl->push_back(pcl::PointXYZ(x,y,0));
      currentpcl->push_back(pcl::PointXYZ(x,y,0));
      current_pcl.push_back(Point(x,y));
    }
  }
  std::sort(current_pcl.begin(),current_pcl.end(),comparePts);

  pcl::io::savePLYFileASCII("currentpcl.ply", *currentpcl);
}

bool isValidPoint(float range, float range_max, float range_min){
  return range < range_max && range > range_min;
}

void filterPCL(PointCloudXY::Ptr mypcl, PointCloudXY::Ptr currentpcl, float vx, float vy, float flappyPosX){
 // for(PointCloudXY::iterator it = mypcl->begin(); it!=mypcl->end(); it++){
  // }
  

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);



  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (int i = 0; i < (*mypcl).size(); i++)
  {
    if ((mypcl->points[i].x- flappyPosX) < 0.0f){
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(mypcl);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*currentpcl);

    //   std::sort(mypcl->points.begin(),mypcl->points.end(), [](pcl::PointXYZ a, pcl::PointXYZ b) {
    //     return a->x > b->x;   
    // });
  // struct {
  //       bool operator()(pcl::PointXYZ a, pcl::PointXYZ b) const
  //       {   
  //           return a.x < b.x;
  //       }   
  //   } customLess;
  //   std::sort(mypcl->points.begin(),mypcl->points.end(), customLess);

  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud(mypcl);
  // pass.setFilterFieldName ("x");
  // pass.setFilterLimits(flappyPosX);
  // pass.filter(*cloud_filtered);
  // mypcl=cloud_filtered;
}

void updateFlappyPos(Point& flappyPos, float vx, float vy){
  float dX = vx/30;
  float dY = vy/30;

  flappyPos.x+=dX;
  flappyPos.y+=dY;
}

double getMiddleOfGap(std::vector<Point>& current_pcl){
  double midY = 0.0;
  double maxGap = 0.0;
  for(int i = 0; i < current_pcl.size()-1;i++){
    double gap = current_pcl.at(i+1).y-current_pcl.at(i).y;
    if(gap>maxGap){
      maxGap=gap;
      midY= 0.5*(current_pcl.at(i+1).y+current_pcl.at(i).y);
    }
  } 
  if(maxGap < 0.3){
    midY = 0.0;
  }
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
