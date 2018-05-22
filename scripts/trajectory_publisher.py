#!/usr/bin/env python

import rospy
import numpy as np
import math 
import tf

from geometry_msgs.msg import Point 
from nav_msgs.msg import GridCells

class Params:
    rate = 10.0
    floor = -0.7
    
class LineRenderer():
    
    def __init__(self):
        self.__desired_route = None

        user_route_sub_topic = rospy.get_param('~nav_route_sub_topic', '/navigation/operator/desired')
        rospy.Subscriber(user_route_sub_topic, GridCells, self.__user_route_callback)
       
        self._pub = rospy.Publisher('/hololens/display/trajectory', GridCells, queue_size=10)
    

    def __user_route_callback(self, msg):
        self.__desired_route = msg.cells
    

    def publish_trajectory(self, position, rotation):    
        msg = GridCells()
        msg.header.stamp = rospy.Time.now()
        theta = -(tf.transformations.euler_from_quaternion(rotation)[2])
        
        if self.__desired_route:

            for cell in self.__desired_route:
                temp = Point()

                x = cell.x-position[0]
                y = cell.y-position[1]

                temp.x = round(math.cos(theta) * x - math.sin(theta) * y, 3)
                temp.y = round(math.sin(theta) * x + math.cos(theta) * y, 3)
                temp.z = Params.floor 
                
                msg.cells.append(temp)

        self._pub.publish(msg)
            
      
# Main loop
def main():
    # Initialize the Formatted Grid node
    rospy.init_node('minimap_processor', anonymous=True)
    lr = LineRenderer()
    
    rate = rospy.Rate(Params.rate)
    
    listener = tf.TransformListener()
    
    while not rospy.is_shutdown():
        try:
            curr_time = rospy.Time(0)
            (position, rotation) = listener.lookupTransform('/odom', '/base_link', curr_time)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        lr.publish_trajectory(position, rotation)
        rate.sleep()
        
# Main function
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
