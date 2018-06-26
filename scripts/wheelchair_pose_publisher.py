#!/usr/bin/env python

import rospy
import numpy as np
import math 
import tf

from geometry_msgs.msg import Pose

class Params:
    rate = 10.0
      
# Main loop
def main():
    # Initialize the Formatted Grid node
    rospy.init_node('wheelchair_publisher_node', anonymous=True)
    

    pub = rospy.Publisher("/hololens/wheelchair_pose", Pose, queue_size=10)
    rate = rospy.Rate(Params.rate)

    
    listener = tf.TransformListener()
    
    while not rospy.is_shutdown():
        if listener.frameExists("Unity"):
            try:
                curr_time = rospy.Time(0)
                (position, rotation) = listener.lookupTransform('/Unity', '/base_link', curr_time)
                
                
                msg = Pose()
                msg.position.x = round(position[0], 3)
                msg.position.y = round(position[1], 3)
                msg.position.z = round(position[2], 3)

                msg.orientation.x = round(rotation[0], 3)                
                msg.orientation.y = round(rotation[1], 3)
                msg.orientation.z = round(rotation[2], 3)
                msg.orientation.w = round(rotation[3], 3)

                pub.publish(msg)
                
                
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        rate.sleep()
        
# Main function
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
