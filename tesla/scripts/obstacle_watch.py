#!/usr/bin/env python
# encoding: utf-8
import rospy

class Obstacle_watch:
    def __init__(self):
        # Initialize node
        rospy.init_node('Obstacle_watch')
        print('Obstacle_watch node is online')

        rospy.on_shutdown(self.cancel)

    def cancel(self):
        print('Closing Node')

if __name__ == '__main__':
    Obstacle_watch()