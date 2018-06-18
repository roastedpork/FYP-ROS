#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import tf

from cv_bridge import CvBridge, CvBridgeError
from multiprocessing import Lock

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells
from sensor_msgs.msg import Image
from hololens_experiment.msg import IntensePixel

class Params:
    rate = 10.0
    blow_up = 14
    deadzone = 0.1
    dilate_factor = 2
    erode_factor = 2
    size = 300   
class Minimap():
    
    def __init__(self):
        self.__bridge = CvBridge()
        
        self.__occ_grid = None
        self.__grid_img = None
        
        self.__desired_route = None

        self.__last_pose_update = rospy.Time.now()

        # Subscribers and Publishers
        occ_grid_sub_topic = rospy.get_param('~occ_grid_sub_topic', '/navigation/operator/local_map/costmap')
        rospy.Subscriber(occ_grid_sub_topic, OccupancyGrid, self.__occ_grid_callback)
        user_route_sub_topic = rospy.get_param('~nav_route_sub_topic', '/navigation/operator/desired')
        rospy.Subscriber(user_route_sub_topic, GridCells, self.__user_route_callback)
       
        self.__image_pub = rospy.Publisher('/minimap/image', Image, queue_size=10)
    

    def __occ_grid_callback(self, msg):
        self.__occ_grid = msg.data
        self.__grid_info = msg.info
        
        # Initialise blank image to be same dimensions as occupancy grid
        if self.__grid_img is None:
            self.__grid_img = np.zeros((msg.info.height, msg.info.width, 3), dtype=np.uint8)
    
    def __user_route_callback(self, msg):
        self.__desired_route = msg.cells
    
    def __apply_rotation(self, img, angle):
        rows, cols, _ = img.shape
        
        rot_mat = cv2.getRotationMatrix2D((cols/2, rows/2), (angle*180.0)/np.pi, 1.0)
        rot_img = cv2.warpAffine(img, rot_mat, (cols, rows), flags=cv2.INTER_LINEAR)
        
        return rot_img
        
    # Overlay desired route of user
    def __overlay_route(self, img, position):
        rows, cols, _ = img.shape
        resolution = self.__grid_info.resolution
        centre_r = rows/2
        centre_c = cols/2
        
        prev_x = None
        prev_y = None

        if self.__desired_route is not None:
            for cell in self.__desired_route:
                metre_x = cell.x-position[0]
                metre_y = cell.y-position[1]
                
                ind_x = int(metre_x/resolution + centre_r) 
                ind_y = int(metre_y/resolution + centre_c)
                
                if not prev_x:
                    prev_x = ind_x
                    prev_y = ind_y
                    continue

                cv2.line(img, (prev_x, prev_y), (ind_x, ind_y), (0,255,0), 2)
                prev_x = ind_x
                prev_y = ind_y
  
        cv2.circle(img, (centre_r, centre_c), 5, (255,0,0), -1)
        return img

    def publish_minimap(self, position, rotation):    
        if self.__occ_grid:
            np_arr = np.array(self.__occ_grid, dtype=np.int8)
            
            # Map occupied cells to white, free to black and everything else to grey
            occupied = (np_arr == 100)
            np_arr[~occupied] = 255
            np_arr[occupied] = 0
            
            # Instantiate numpy array for image
            img = np.ndarray(shape=(self.__grid_info.height, self.__grid_info.width),
                           dtype=np.uint8, buffer=np_arr)
                         
            # Invert image
            img = 255 - img
            
            # Dilate and erode white for expanded border dimensions
            # Morphological process also closes small holes between scanner readings
            kernel = np.ones((3,3), np.uint8)
            img = cv2.dilate(img, kernel, iterations=Params.dilate_factor)
            img = cv2.erode(img, kernel, iterations=Params.erode_factor)

            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            img = self.__overlay_route(img, position)

            img = cv2.GaussianBlur(img, (3,3), 0)
            theta = tf.transformations.euler_from_quaternion(rotation)[2] + np.pi/2 
            img = self.__apply_rotation(img, theta)
            img = cv2.flip(img, 1)
            img = cv2.resize(img, (Params.size, Params.size), interpolation = cv2.INTER_LINEAR)

            try:
                final_img = self.__bridge.cv2_to_imgmsg(img, 'bgr8')
            except CvBridgeError as e:
                print(e)

            final_img.header.stamp = rospy.Time.now()
            self.__image_pub.publish(final_img)

            
      
# Main loop
def main():
    # Initialize the Formatted Grid node
    rospy.init_node('minimap_processor', anonymous=True)
    mm = Minimap()
    
    rate = rospy.Rate(Params.rate)
    
    listener = tf.TransformListener()
    
    while not rospy.is_shutdown():
        try:
            curr_time = rospy.Time(0)
            (position, rotation) = listener.lookupTransform('/odom', '/base_link', curr_time)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        mm.publish_minimap(position, rotation)
        rate.sleep()
        
# Main function
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
