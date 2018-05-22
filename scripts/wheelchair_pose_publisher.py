#!/usr/bin/env python

import rospy
import numpy as np
import math 
import tf

from geometry_msgs.msg import Pose2D

class Params:
    rate = 10.0
    floor = -0.7
      
# Main loop
def main():
    # Initialize the Formatted Grid node
    rospy.init_node('minimap_processor', anonymous=True)
    
    rate = rospy.Rate(Params.rate)

    
    listener = tf.TransformListener()
    
    while not rospy.is_shutdown():
        try:
            curr_time = rospy.Time(0)
            (position, rotation) = listener.lookupTransform('/odom', '/base_link', curr_time)
            
            theta = tf.transformations.euler_from_quaternion(rotation)[2]
            
            msg = Pose2D()
            msg.x = round(position[0], 3)
            msg.y = round(position[1], 3)
            msg.theta = round(theta, 3)
            
            
            
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rate.sleep()
        
# Main function
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
