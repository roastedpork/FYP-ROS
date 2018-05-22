#!/usr/bin/env python

import rospy
import numpy as np
import copy
import math
import cv2
import tf

from cv_bridge import CvBridge, CvBridgeError
from multiprocessing import Lock

from nav_msgs.msg import OccupancyGrid, GridCells
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from hololens_project.msg import Obstacle, ObstacleArray

class Params:
    rate = 10.0
    scan_range = 7.0
    use_local_map = True
    floor_height = -1.0
    debug = False
    filter_thresh = 5

    min_obs_size = 100.0
    num_iter = 1
    dilate_iter = 2
    erode_iter = 2
    
    gauss_kernel = 5
    k_size = 3




class ObstacleDetection():
    
    def __init__(self):
        self.bridge = CvBridge()
        
        self.occ_grid = None
        self.grid_pose = None
        self.grid_img = None
        
        self.desired_route = None


        self.user_cmd = None
        self.user_cmd_lock = Lock()
        # Subscribers and Publishers
        if Params.use_local_map:
            occ_grid_sub_topic = rospy.get_param('~occ_grid_sub_topic', '/navigation/operator/local_map/costmap')
        else:
            occ_grid_sub_topic = rospy.get_param('~occ_grid_sub_topic', '/navigation/map')
        rospy.Subscriber(occ_grid_sub_topic, OccupancyGrid, self.occ_grid_callback)

        self.obs_debug_pub = rospy.Publisher("/formatted_grid/obs_debug_image", Image, queue_size=10)
        
        self.obs_pub = rospy.Publisher("/formatted_grid/obs_array", ObstacleArray, queue_size=10)


    def occ_grid_callback(self, msg):
        self.occ_grid = msg.data
        self.grid_info = msg.info
        
        # Initialise blank image to be same dimensions as occupancy grid
        if self.grid_img is None:
            self.grid_img = np.zeros((msg.info.height, msg.info.width, 3), dtype=np.uint8)
    
    def apply_rotation(self, img, angle):
        rows, cols, _ = img.shape
        rot_mat = cv2.getRotationMatrix2D((cols/2, rows/2), (angle*180.0)/np.pi, 1.0)
        rot_img = cv2.warpAffine(img, rot_mat, (cols, rows), flags=cv2.INTER_LINEAR)

        return rot_img

    def clamp_angle(self,theta):
        if theta > np.pi: theta -= 2*np.pi
        if theta <-np.pi: theta += 2*np.pi
        return theta

    def publish_obs_array(self, position, rotation):    
        if self.occ_grid is not None:

            np_arr = np.array(self.occ_grid, dtype=np.int8)
            
            # Map occupied cells to white, free to black and everything else to grey
            occupied = (np_arr == 100)
            #unknown = (np_arr < 0)
           
            np_arr[~occupied] = 255
            np_arr[occupied] = 0

            # Instantiate numpy array for image
            img = np.ndarray(shape=(self.grid_info.height, self.grid_info.width),
                           dtype=np.uint8, buffer = np_arr)

            centre_r = self.grid_info.height/2
            centre_c = self.grid_info.width/2
            if not Params.use_local_map:
                ind_range = int(Params.scan_range/self.grid_info.resolution + 0.5)

                minR, maxR = centre_r - ind_range, centre_r + ind_range
                minC, maxC = centre_c - ind_range, centre_c + ind_range
                img = img[minR:maxR, minC:maxC]

            # Invert image
            img = 255-img
           
            
            kernel = np.ones((Params.k_size, Params.k_size), np.uint8)
            # Performing obstacle detection
            for _ in range(Params.num_iter):
                img = cv2.dilate(img, kernel, iterations = Params.dilate_iter)
                img = cv2.GaussianBlur(img, (Params.gauss_kernel, Params.gauss_kernel), 0)
                img = cv2.erode(img, kernel, iterations = Params.erode_iter)

            ret, thresh = cv2.threshold(img, Params.filter_thresh,255, cv2.THRESH_BINARY)
            im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)

            theta = tf.transformations.euler_from_quaternion(rotation)[2] # + np.pi/2
            

            obs_list = ObstacleArray()
            obs_list.header.stamp = rospy.Time.now()

            for i,cnt in enumerate(contours):
                area = cv2.contourArea(cnt)
                if area > Params.min_obs_size:
                    temp = Obstacle()

                    # Find min bounding box of filtered obstacle
                    rect = cv2.minAreaRect(cnt)
                    (cX,cY), (w,h), angle = rect

                    box = np.int0(cv2.boxPoints(rect))
                    cv2.circle(img, (int(cX),int(cY)), 3, (255,0,0),-1)
                    cv2.drawContours(img, [box], -1, (0,255,0), 1)
                   
                    # Calculate distance and angle of obstacle from wheelchair
                    dX_w = (cX - centre_c) * self.grid_info.resolution
                    dY_w = (cY - centre_r) * self.grid_info.resolution

                    dist = np.linalg.norm(np.array([dX_w,dY_w]))
                    rel_angle = self.clamp_angle(math.atan2(dY_w,dX_w)-theta)

                    dX = dist * math.cos(rel_angle)
                    dY = dist * math.sin(rel_angle)

                    w *= self.grid_info.resolution
                    h *= self.grid_info.resolution

                    box_angle_w = np.pi/2 - abs(np.radians(angle))
                    box_angle_rel = self.clamp_angle(box_angle_w - theta)



                    # Draw obstacle on image
                    #label = "(%1.3f, %1.3f, %1.3f)" % (dX, dY, angle)
                    #label = "(%1.3f, %1.3f), %1.3f" % (w, h ,angle)
                    #cv2.putText(img, label, (int(cX)-10,int(cY)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
                
                    # Add obstacle to publishing message
                    
                    temp.rel_position.x = round(dX,3)
                    temp.rel_position.y = round(dY,3)
                    temp.rel_position.z = round(Params.floor_height,3)
                    temp.width = round(w,3)
                    temp.height = round(h,3)
                    temp.box_angle = round(box_angle_rel,3)
                    obs_list.obstacles.append(temp)


            # Draw wheelchair
            cv2.circle(img, (centre_r,centre_c), 3, (255,0,255), -1)

            if not Params.debug:
                img = self.apply_rotation(img, theta + np.pi/2)
                img = cv2.flip(img,1)

            # Publish Obstacle Debug Map
            try:
                final_image = self.bridge.cv2_to_imgmsg(img, 'rgb8')
                final_image.header.stamp = rospy.Time.now()

                self.obs_debug_pub.publish(final_image) 
            except CvBridgeError as e:
                print(e)
            
            self.obs_pub.publish(obs_list)


# oop
def main():
    # Initialize the Formatted Grid node
    rospy.init_node('gridmap_processor', anonymous=True)
    od = ObstacleDetection()
    
    rate = rospy.Rate(Params.rate)
    
    listener = tf.TransformListener()
    
    while not rospy.is_shutdown():
        try:
            curr_time = rospy.Time(0)
            (position, rotation) = listener.lookupTransform('/odom', '/base_link', curr_time)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        od.publish_obs_array(position, rotation)
        rate.sleep()
        
# Main function
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
