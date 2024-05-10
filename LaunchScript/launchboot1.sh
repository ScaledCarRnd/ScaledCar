#!/bin/bash

# Open script in a new terminal window
# gnome-terminal -- /home/jetson/catkin_ws/src/LaunchScript/launchboot.sh

# Source ROS setup file
source /home/jetson/catkin_ws/devel/setup.bash

# Change directory to the home directory
cd /home/jetson/catkin_ws/src/LaunchScript

# Paths and commands
LIVEDAY_LAUNCH="/home/jetson/catkin_ws/src/tesla/tesla.launch"
OBSTACLE_TEST_LAUNCH="tesla obstacle_test.launch"
CAMTEST_PY="tesla camtest.py"
HELLO_WORLD_PY="/home/jetson/Desktop/LaunchScript/hello_world.py"

# Variable to store the PID of the launched process
# This doesn't work, can't figure out why but it's not necessary
LAUNCHED_PROCESS_PID=""

# Function to display menu
display_menu() {
    # gnome-terminal & #Hopefully fixes the systemd launch file
    clear
    echo
    echo " ------------------------"
    echo -e " \e[1m\e[32mROS Launch File Selector\e[0m"  # Change text color to green and make it bold
    echo " ------------------------"
    echo
    echo "  Please input the following numerical choices (1-3)"
    echo "  1. AUT Live Day (Recommended)"
    echo "  2. *BETA* Obstacle Detection Version with Camera"
    # echo "  3. Launch demo Python Script"
    echo "  3. Exit"
    echo
}
# For this display_menu function I guess we could launch important automonous launch files and visual display stuff individually?


# Function to run selected option
run_option() {
    case $1 in

        # gnome-terminal opens each choice in a new terminal
	1) gnome-terminal -- bash -c "roslaunch $LIVEDAY_LAUNCH 2>/home/jetson/Desktop/error.txt" & LAUNCHED_PROCESS_PID=$!;;
        2) gnome-terminal -- bash -c "roslaunch $OBSTACLE_TEST_LAUNCH" && sleep 10 && gnome-terminal -- bash -c "rosrun $CAMTEST_PY" & LAUNCHED_PROCESS_PID=$!;;
        # 3) gnome-terminal -- python3 $HELLO_WORLD_PY & LAUNCHED_PROCESS_PID=$!;;
        3) echo " Exiting..."; exit;;
        *) echo " Invalid option";;
    esac
}

# Function to terminate the launched process
terminate_launched_process() {
    if [ -z "$LAUNCHED_PROCESS_PID" ]; then
        echo "No process is currently running."
    else
        echo "Terminating the launched process..."
        kill $LAUNCHED_PROCESS_PID
        LAUNCHED_PROCESS_PID=""
    fi
}

# Main function
main() {
    while true; do
        display_menu

        read -rp " Enter your choice: " choice

        if [[ ! $choice =~ ^[1-3]$ ]]; then
            echo "Invalid option. Please enter a number between 1 and 4."
        else
            run_option $choice
            read -rp "Press Enter to continue or type 'x' to quit: " input
            # The input is case-sensitive
            if [ "${input,,}" = "x" ]; then
			terminate_launched_process
                echo "Exiting..."
				sleep 5
                exit
            fi
        fi
    done
}

# Call the main function
#main
