//Author: Vishwa Theja Pokala
//Reference: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h> 

#define PI 3.14159265359

int main(int argc, char** argv){
  ros::init(argc, argv, "fake_laser_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);


  //the following values are based on Hokuyo UST-20LX laser scanner and http://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg/

  unsigned int num_readings = 1081;
  double laser_frequency = 50;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  srand(time(0));
  ros::Rate r(1.0);
  while(n.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = 4 - (0.04*double(rand())/double(RAND_MAX));//assuming a wall thats 4 m surrounding the robot from all sides 
      intensities[i] = 10;
    }
    ros::Time scan_time = ros::Time::now();

    //Assuming a lidar with 1081 measurement steps, detection angle of 270 Degrees and angular resolution of 0.25 Degrees
    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "base_laser";
    scan.angle_min = -135 * (PI/180); //angle correspond to FIRST beam in scan ( in rad)
    scan.angle_max = 135 * (PI/180); //angle correspond to LAST beam in scan ( in rad)
    scan.angle_increment = 0.25 * (PI/180); // Angular resolution i.e angle between 2 beams
    scan.time_increment = (1 / 50) / (1081); 
    scan.range_min = 0.02;
    scan.range_max = 20.0;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }
    
    scan_pub.publish(scan);
    r.sleep();
    
  }
}
