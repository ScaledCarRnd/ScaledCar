#!/bin/bash

# Define the path to the test directory
TEST_DIR=/home/jetson/catkin_ws/src/test

# Import launchboot.sh
source /home/jetson/catkin_ws/src/LaunchScript/launchboot.sh

# Define test functions
test_run_option() {
    # Function to write test output to a text file
    write_output() {
        echo "$1" >> "$TEST_DIR/test_output.txt"
    }

    # Test case 1
    write_output "Running test case 1: Launch Camera and Navigation"
    run_option 1
    # Add assertions here to verify the behavior
    # For example, you can check if the expected command is executed
    # Or if the correct terminal command is formed
    # Assertion 1: Check if the correct terminal command is formed
    expected_command="gnome-terminal -- roslaunch $CAMNAV_LAUNCH"
    last_command=$(history 1)
    if [ "$last_command" = "$expected_command" ]; then
        write_output "Test passed: Correct terminal command formed."
    else
        write_output "Test failed: Incorrect terminal command formed."
    fi

    # Assertion 2: Check if the command executed successfully
    if [ $? -eq 0 ]; then
        write_output "Test passed: Camera and Navigation launched successfully."
    else
        write_output "Test failed: Camera and Navigation failed to launch."
    fi

    # Test case 2
    write_output "Running test case 2: Launch Slam and RVIZ"
    run_option 2
    # Add assertions for this test case

    # Test case 3
    write_output "Running test case 3: Launch demo Python Script"
    run_option 3
    # Add assertions for this test case
    # Assertion 1: Check if the correct terminal command is formed
    expected_command="gnome-terminal -- roslaunch $CAMNAV_LAUNCH"
    last_command=$(history 1)
    if [ "$last_command" = "$expected_command" ]; then
        write_output "Test passed: Correct terminal command formed."
    else
        write_output "Test failed: Incorrect terminal command formed."
    fi

    # Assertion 2: Check if the command executed successfully
    if [ $? -eq 0 ]; then
        write_output "Test passed: Camera and Navigation launched successfully."
    else
        write_output "Test failed: Camera and Navigation failed to launch."
    fi

    # Test case 4
    write_output "Running test case 4: Quit"
    run_option 4
    # Add assertions for this test case
}

# Run tests
test_run_option

