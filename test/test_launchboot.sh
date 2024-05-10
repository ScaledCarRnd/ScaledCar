#!/bin/bash

# Source your script
source /home/jetson/catkin_ws/src/LaunchScript/launchboot1.sh

# Source ROS setup file
source /home/jetson/catkin_ws/devel/setup.bash

. /home/jetson/catkin_ws/src/test/shunit2/shunit2

# Mock gnome-terminal to just echo the command it would run
gnome-terminal() {
  echo "$@"
}

testRunOption() {
  expected="-- bash -c roslaunch /home/jetson/catkin_ws/src/tesla/tesla.launch 2>/home/jetson/Desktop/error.txt & LAUNCHED_PROCESS_PID=$!"
  actual=$(run_option 1)
  assertEquals "Command did not match expected" "$expected" "$actual"
}

# Load shUnit2.
. /home/jetson/catkin_ws/src/test/shunit2/shunit2

