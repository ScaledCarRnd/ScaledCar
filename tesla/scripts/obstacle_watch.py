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

        self.costmap_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.local_costmap_callback)
        #self.costmap_pub = rospy.Publisher('/modified_costmap', OccupancyGrid, queue_size=10)
        self.sub_obstacle = rospy.Subscriber("obstacles", obstacleData, self.obstacle_callback)
        self.costmap_pub = rospy.Publisher('obstalce', OccupancyGrid, queue_size=10)
        
        self.modified_costmap = None
        
        self.map_size = 100
        # own costmap
        self.transform_offset = Point()
        self.transform_offset.x = -5
        self.transform_offset.y = -5
        self.transform_offset.z = 0
        self.costmap_msg = OccupancyGrid()
        self.costmap_msg.header.stamp = rospy.Time.now()
        self.costmap_msg.header.frame_id = 'map'  # Set the frame ID
        self.costmap_msg.info.resolution = 0.1  # Set the resolution in meters
        self.costmap_msg.info.width = self.map_size  # Set the width of the costmap (number of cells)
        self.costmap_msg.info.height = self.map_size  # Set the height of the costmap (number of cells)
        self.costmap_msg.info.origin.position.x = self.transform_offset.x  # Set the origin of the costmap
        self.costmap_msg.info.origin.position.y = self.transform_offset.y
        self.costmap_msg.info.origin.position.z = 0

        # mapsize * resolution give real map width/height (100 size / 0.1 resolution -> 10m)
        # divide that by mapsize to get distance per cell (10 / 100 -> 0.1m per cell)
        self.map_world_size = self.map_size * self.costmap_msg.info.resolution
        self.worl_distance_per_cell = self.map_world_size / self.map_size 

        # Initialize the occupancy grid data (all cells are free)
        self.costmap_msg.data = [0] * (self.costmap_msg.info.width * self.costmap_msg.info.height)
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

    def local_costmap_callback(self, local_costmap):
        # Store received costmap
        self.modified_costmap = local_costmap
        # Update the origin of the costmap based on the local costmap's origin
        # This will translate the whole map as the robot moves 
        self.costmap_msg.info.origin = local_costmap.info.origin
        rospy.loginfo(rospy.get_caller_id() + "Updated Costmap Origin--> X: %d Y: %d", 
                      self.costmap_msg.info.origin.position.x, self.costmap_msg.info.origin.position.y)


    def obstacle_callback(self, msg):
        #get the obstacle location in centimetres, change to metres
        obstacle_x = msg.x / 100.0
        obstacle_y = msg.y / 100.0


        rospy.loginfo(rospy.get_caller_id() + "Costmap Origin--> X: %d Y: %d" , self.costmap_msg.info.origin.position.x, self.costmap_msg.info.origin.position.y)
        rospy.loginfo(rospy.get_caller_id() + "Original values--> X: %f Y: %f" , obstacle_x, obstacle_y)
        
        # Subtract the origin of the costmap from the obstacle position, normalize for grid offset
        obstacle_x = obstacle_x - self.costmap_msg.info.origin.position.x
        obstacle_y = obstacle_y - self.costmap_msg.info.origin.position.y
        rospy.loginfo(rospy.get_caller_id() + "Normalized --> X: %f Y: %f" , obstacle_x, obstacle_y)

        # Convert into costmap index
        obstacle_x = int(obstacle_x / self.map_world_size * self.map_size)
        obstacle_y = int(obstacle_y / self.map_world_size * self.map_size)
        rospy.loginfo(rospy.get_caller_id() + "Indexed --> X: %d Y: %d" , obstacle_x, obstacle_y)

        rospy.loginfo(rospy.get_caller_id() + "Transformed values--> X: %d Y: %d" , obstacle_x, obstacle_y)

        #Fake dimensions
        obstacle_w = 10
        obstacle_h = 6

        for y in range(obstacle_y - obstacle_w / 2, obstacle_y + obstacle_w / 2):
            for x in range(obstacle_x - obstacle_h / 2, obstacle_x + obstacle_h / 2):
                rospy.loginfo(rospy.get_caller_id() + "Recorded values--> X: %d Y: %d" , x, y)
                if 0 <= x < self.costmap_msg.info.width and 0 <= y < self.costmap_msg.info.height:
                    index = y * self.costmap_msg.info.width + x
                    self.costmap_msg.data[index] = 100  # Set occupancy value to 100 for occupied cells



        # obstacle_x = int(x * self.costmap_msg.info.resolution + self.costmap_msg.info.origin_x)
        # obstacle_y = int(y * self.costmap_msg.info.resolution + self.costmap_msg.info.origin_y)
        # #test
        # ##########################################
        # for y in range(30, 70):  # Rows
        #     for x in range(30, 40):  # Columns
        #         index = y * self.costmap_msg.info.width + x
        #         self.costmap_msg.data[index] = random.randint(0, 100)
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
        # if self.modified_costmap is not None:
        #     self.update_costmap(obstacle_position)
        # rospy.loginfo(rospy.get_caller_id() + " Updated self costmap with obstacle data")

    # Calculate grid coordinates of obstacle position
    # Update occupancy values in costmap to mark cells as occupied
    # Modify self.modified_costmap, and publish
    # def update_costmap(self, obstacle_position):
        
    #     grid_x = int((obstacle_position.x - self.modified_costmap.info.origin.position.x) / self.modified_costmap.info.resolution)
    #     grid_y = int((obstacle_position.y - self.modified_costmap.info.origin.position.y) / self.modified_costmap.info.resolution)

    #     if 0 <= grid_x < self.modified_costmap.info.width and 0 <= grid_y < self.modified_costmap.info.height:
    #         index = grid_y * self.modified_costmap.info.width + grid_x
    #         self.modified_costmap.data[index] = 100  # Set occupancy value to 100 for occupied cells

    #     # Publish modified costmap
    #     self.costmap_pub.publish(self.modified_costmap)
    #     rospy.loginfo(rospy.get_caller_id() + " Published costmap to modified_costmap")

if __name__ == '__main__':
    Obstacle_watch()