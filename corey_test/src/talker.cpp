#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");

  ros::Rate loop_rate(10);
 
  while(ros::ok())
  {
    loop_rate.sleep();
 
  }

	
  return 0;
}
