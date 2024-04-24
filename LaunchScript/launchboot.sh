#!/bin/bash

# Paths and commands
CAMNAV_LAUNCH="jetracer run_camnav.launch"
NAVMAP_LAUNCH="jetracer run_navmap.launch"
HELLO_WORLD_PY="/home/jetson/Desktop/LaunchScript/hello_world.py"

# Function to display menu
display_menu() {
    clear
    echo
    echo " ------------------------"
    echo -e " \e[1m\e[32mROS Launch File Selector\e[0m"  # Change text color to green and make it bold
    echo " ------------------------"
    echo
    echo "  1. Launch Camera and Navigation"
    echo "  2. Launch Slam and RVIZ"
    echo "  3. Launch demo Python Script"
    echo "  4. Quit"
    echo
}
# For this display_menu function I guess we could launch important automonous launch files and visual display stuff individually?


# Function to run selected option
run_option() {
    case $1 in
        # gnome-terminal opens each choice in a new terminal
	1) gnome-terminal -- roslaunch $CAMNAV_LAUNCH;;
        2) gnome-terminal -- roslaunch $NAVMAP_LAUNCH;;
        3) gnome-terminal -- python3 $HELLO_WORLD_PY;;
        4) echo " Exiting..."; exit;;
        *) echo " Invalid option";;
    esac
}

# Main function
main() {
    while true; do
        display_menu
        read -rp " Enter your choice: " choice
        if [[ ! $choice =~ ^[1-4]$ ]]; then
            echo "Invalid option. Please enter a number between 1 and 4."
        else
            run_option $choice
            read -rp "Press Enter to continue or type 'x' to quit: " input
            # The input is case-sensitive
            if [ "${input,,}" = "x" ]; then
                echo "Exiting..."
                exit
            fi
        fi
    done
}

# Call the main function
main
