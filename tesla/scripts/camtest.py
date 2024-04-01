#!/usr/bin/env python
# encoding: utf-8
import rospy
import sys
import argparse
from jetson_inference import imageNet
from jetson_utils import videoSource, videoOutput, cudaFont, Log

class Camtest:
    def __init__(self):
        # Initialize node
        rospy.init_node('Cam_Test')
        print('Cam_Test node is online')

        #Do Stuff


        # Destructor
    def cancel(self):
        print('Closing Node')


if __name__ == '__main__':
    Camtest()