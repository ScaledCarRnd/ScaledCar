#!/bin/bash

# Function to display menu
display_menu() {
    clear
    echo
    echo " ------------------------"
    echo -e " \e[1m\e[32mROS Launch File Selector\e[0m"  # Change text color to green and make it bold
    echo " ------------------------"
    echo
    echo "  1. Launch File 1"
    echo "  2. Launch File 2"
    echo "  3. Launch demo Python Script"
    echo "  4. Quit"
    echo
}
# For this display_menu function I guess we could launch important automonous launch files and visual display stuff individualy?


# Function to run selected option
run_option() {
    case $1 in
        # 1) roslaunch package_name launch_file_1.launch;;
	1) roslaunch jetracer run_camnav.launch;;
        2) roslaunch package_name launch_file_2.launch;;
        3) python3 /home/jetson/Desktop/LaunchScript/hello_world.py;;
        4) echo " Exiting..."; exit;;
        *) echo " Invalid option";;
    esac
}

# Main function
main() {
    while true; do
        display_menu
        read -p " Enter your choice: " choice
        run_option $choice
        read -p " Press Enter to continue..."
    done
}

# Call the main function
main

