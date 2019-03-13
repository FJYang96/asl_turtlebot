#!/usr/bin/env python

import rospy
import math
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Pose2D, PoseStamped
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from asl_turtlebot.msg import DetectedObjectList, ObjectLocation
from std_msgs.msg import Header, Float32MultiArray

class foodlocation:
    def __init__(self):
        rospy.init_node('food_location_marker')
        while rospy.Time().now() == rospy.Time(0):
            pass

        self.food_list = []
        self.food_x = []
        self.food_y = []

        self.foods_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size = 10)
        rospy.Subscriber('/food_location', ObjectLocation, self.food_location_callback)

    def food_location_callback(self, msg):
        rospy.loginfo("Food location received!")
        self.food_list = msg.name
        self.food_x = msg.x
        self.food_y = msg.y

    def loop(self):
        if len(self.food_list) == 0:
            return
        foods_marker = MarkerArray()
        for i in range(len(self.food_list)):
            food_marker = Marker()
            food_marker.header.frame_id = "/map"
            food_marker.header.stamp = rospy.Time(0)
            food_marker.ns = "food"
            food_marker.action = food_marker.ADD
            food_marker.type = food_marker.TEXT_VIEW_FACING
            food_marker.pose.position.x = self.food_x[i]
            food_marker.pose.position.y = self.food_y[i]
            food_marker.color.a = 1.0
            food_marker.color.r = 1.0
            food_marker.scale.z = 0.25
            food_marker.text = self.food_list[i]
            food_marker.id = i
            food_marker.lifetime = rospy.Duration(10)
            foods_marker.markers.append(food_marker)
        
        self.foods_pub.publish(foods_marker)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = foodlocation()
    sup.run()
