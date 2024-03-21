#!/usr/bin/env python
# encoding: utf-8
import random
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from tesla.msg import obstacleData


class Obstacle_watch:
    def __init__(self):
        # Initialize node
        rospy.init_node('Obstacle_watch')
        print('Obstacle_watch node is online')

        self.costmap_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_callback)
        self.costmap_pub = rospy.Publisher('/modified_costmap', OccupancyGrid, queue_size=10)
        self.sub_obstacle = rospy.Subscriber("obstacles", obstacleData, self.obstacle_callback)
        self.costmap_pub = rospy.Publisher('/custom_costmap', OccupancyGrid, queue_size=10)
        self.modified_costmap = None
        # own costmap
        self.costmap_msg = OccupancyGrid()
        self.costmap_msg.header.stamp = rospy.Time.now()
        self.costmap_msg.header.frame_id = 'map'  # Set the frame ID
        self.costmap_msg.info.resolution = 0.1  # Set the resolution in meters
        self.costmap_msg.info.width = 100  # Set the width of the costmap (number of cells)
        self.costmap_msg.info.height = 100  # Set the height of the costmap (number of cells)
        self.costmap_msg.info.origin.position.x = -5  # Set the origin of the costmap
        self.costmap_msg.info.origin.position.y = -5
        self.costmap_msg.info.origin.position.z = 0

        # Initialize the occupancy grid data (all cells are free)
        self.costmap_msg.data = [0] * (self.costmap_msg.info.width * self.costmap_msg.info.height)

        # Add some obstacles (for testing purposes)
        for y in range(40, 60):  # Rows
            for x in range(40, 60):  # Columns
                index = y * self.costmap_msg.info.width + x
                self.costmap_msg.data[index] = 100  # Set occupancy value to 100 for occupied

        self.costmap_msg.header.stamp = rospy.Time.now()  # Update timestamp
        self.costmap_pub.publish(self.costmap_msg)  # Publish the costmap message

        # Set destructer
        rospy.on_shutdown(self.cancel)

        # Main Loop
        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            
            rate.sleep()

    # Destructor
    def cancel(self):
        print('Closing Node')

    def costmap_callback(self, costmap):
        # Store received costmap
        self.modified_costmap = costmap

    def obstacle_callback(self, msg):
        #test
        ##########################################
        for i in range(len(self.costmap_msg.data)):
            self.costmap_msg.data[i] = random.randint(0, 100)
        self.costmap_msg.header.stamp = rospy.Time.now()  # Update timestamp
        self.costmap_pub.publish(self.costmap_msg)  # Publish the costmap message
        ##########################################

        print('Obstacle callback')
        rospy.loginfo(rospy.get_caller_id() + ": Obstacle Data Recieved!")
        rospy.loginfo(rospy.get_caller_id() + ": String Data: %s", msg.strData)
        rospy.loginfo(rospy.get_caller_id() + ": ID: %d", msg.id)
        rospy.loginfo(rospy.get_caller_id() + ": X: %d", msg.x)
        rospy.loginfo(rospy.get_caller_id() + ": Y: %d", msg.y)
        rospy.loginfo(rospy.get_caller_id() + ": Z: %d", msg.z)

        # Extract obstacle position from message
        obstacle_position = Point()
        obstacle_position.x = msg.x
        obstacle_position.y = msg.y
        obstacle_position.z = msg.z

        # Update modified costmap
        if self.modified_costmap is not None:
            self.update_costmap(obstacle_position)
        rospy.loginfo(rospy.get_caller_id() + " Updated self costmap with obstacle data")

    # Calculate grid coordinates of obstacle position
    # Update occupancy values in costmap to mark cells as occupied
    # Modify self.modified_costmap, and publish
    def update_costmap(self, obstacle_position):
        
        grid_x = int((obstacle_position.x - self.modified_costmap.info.origin.position.x) / self.modified_costmap.info.resolution)
        grid_y = int((obstacle_position.y - self.modified_costmap.info.origin.position.y) / self.modified_costmap.info.resolution)

        if 0 <= grid_x < self.modified_costmap.info.width and 0 <= grid_y < self.modified_costmap.info.height:
            index = grid_y * self.modified_costmap.info.width + grid_x
            self.modified_costmap.data[index] = 100  # Set occupancy value to 100 for occupied cells

        # Publish modified costmap
        self.costmap_pub.publish(self.modified_costmap)
        rospy.loginfo(rospy.get_caller_id() + " Published costmap to modified_costmap")

if __name__ == '__main__':
    Obstacle_watch()