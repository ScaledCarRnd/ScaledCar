#!/usr/bin/env python
# encoding: utf-8
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from tesla.msg import obstacleData


class Obstacle_watch:
    def __init__(self):
        # Initialize node
        rospy.init_node('Obstacle_watch')
        print('Obstacle_watch node is online')

        self.costmap_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_callback)
        self.obstacle_sub = rospy.Subscriber('/obstacle_cloud', PointCloud2, self.obstacle_cloud_callback)
        self.costmap_pub = rospy.Publisher('/modified_costmap', OccupancyGrid, queue_size=10)
        self.sub_obstacle = rospy.Subscriber("obstacles", obstacleData, self.obstacle_callback)


        # Set destructer
        rospy.on_shutdown(self.cancel)

        # Main Loop
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

    # Destructor
    def cancel(self):
        print('Closing Node')

    def costmap_callback(self, costmap):
        # Store received costmap
        self.modified_costmap = costmap

    def obstacle_cloud_callback(self, obstacle_cloud):
        # TODO:
        # Process obstacle point cloud and update modified costmap
        # Modify self.modified_costmap accordingly

        # Publish modified costmap
        if self.modified_costmap is not None:
            self.costmap_pub.publish(self.modified_costmap)

    def obstacle_callback(self, msg):
        print('Obstacle callback')
        rospy.loginfo(rospy.get_caller_id() + ": Obstacle Data Recieved!")
        rospy.loginfo(rospy.get_caller_id() + ": String Data: %s", msg.strData)
        rospy.loginfo(rospy.get_caller_id() + ": ID: %d", msg.id)
        rospy.loginfo(rospy.get_caller_id() + ": X: %d", msg.x)
        rospy.loginfo(rospy.get_caller_id() + ": Y: %d", msg.y)
        rospy.loginfo(rospy.get_caller_id() + ": Z: %d", msg.z)

if __name__ == '__main__':
    Obstacle_watch()