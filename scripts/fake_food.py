#!/usr/bin/env python

import rospy
import numpy as np
from asl_turtlebot.msg import ObjectLocation, DetectedObjectList, DetectedObject

def food_sender():
    rospy.init_node('fake_object')
    fake_food_pub = rospy.Publisher('/detector/objects', DetectedObjectList, queue_size = 10)

    fake_food1 = DetectedObject()
    fake_food1.id = 0
    fake_food1.name = 'pizza'
    fake_food1.confidence = 0.5
    fake_food1.distance = 1.
    fake_food1.thetaleft = np.pi / 2
    fake_food1.thetaright = np.pi / 2
    fake_food1.corners = [0.,0.,0.,0.]

    fake_food2 = DetectedObject()
    fake_food2.id = 0
    fake_food2.name = 'trash'
    fake_food2.confidence = 0.5
    fake_food2.distance = 3.
    fake_food2.thetaleft = -np.pi / 2
    fake_food2.thetaright = -np.pi / 2
    fake_food2.corners = [0.,0.,0.,0.]

    fake_foods = DetectedObjectList()
    fake_foods.objects = ['pizza', 'trash']
    fake_foods.ob_msgs = [fake_food1, fake_food2]

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        fake_food_pub.publish(fake_foods)
        rate.sleep()
        

if __name__ == '__main__':
    
    try:
        food_sender()
    except rospy.ROSInterruptException:
        pass
