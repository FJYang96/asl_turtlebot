#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject, ObjectLocation, DetectedObjectList
from sensor_msgs.msg import Image, CameraInfo, LaserScan
import tf
import math
import numpy as np
import collections

# # if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# # otherwise, they will use a TF lookup (hw2+)
# use_gazebo = rospy.get_param("sim")

# # how is nav_cmd being decided -- human manually setting it, or rviz
# rviz = rospy.get_param("rviz")

# # if using gmapping, you will have a map frame. otherwise it will be odom frame
# mapping = rospy.get_param("map")


class food:

    def __init__(self):
        rospy.init_node('food_location', anonymous=True)
        # initialize variables
        self.trans_listener = tf.TransformListener()
        self.name = []
        self.data = []
        self.new_name = []
        self.new_data = []
        self.count = 0

        self.foodDict_x = collections.OrderedDict()    # key is new_name, value is x tuple
        self.foodDict_y = collections.OrderedDict()
        self.foodDict_count = collections.OrderedDict()

        self.thres = 0.00               # unit in m

        self.trans_listener = tf.TransformListener()
        
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.objects_detected_callback)

        # Size of objects in the real world
        self.world_size = {}
        self.world_size['orange'] = (0.17, 0.12)
        self.world_size['pizza'] = (0.09, 0.075)
        self.world_size['banana'] = (0.09, 0.06)
        self.world_size['stop sign'] = (0.064, 0.064)
        self.world_size['bottle'] = (0.065, 0.205)

        # Camera Parameters for distance estimation
        self.cx, self.cy, self.fx, self.fy = 0, 0, 1, 1
        rospy.Subscriber('/camera_relay/camera_info', CameraInfo, self.camera_info_callback)

        self.food_location_pub = rospy.Publisher('/food_location', ObjectLocation, queue_size=10)

        self.avg = False

    def camera_info_callback(self, msg):
        self.cx = msg.P[2]
        self.cy = msg.P[6]
        self.fx = msg.P[0]
        self.fy = msg.P[5]
        
    def objects_detected_callback(self, msg):
        self.new_name = msg.objects
        self.new_data = msg.ob_msgs
        self.count += 1
        self.calculation()
        self.output()

    def estimate_distance(self, msg):
        range_dist = msg.distance
        # Apparently it is much more accurate to use vision to estimate distance
        if msg.name in self.world_size:
            ymin, xmin, ymax, xmax = msg.corners
            x_pixel_size = np.abs(xmax - xmin)
            y_pixel_size = np.abs(ymax - ymin)
            x_world_size, y_world_size = self.world_size[msg.name]
            visual_distance_from_x = self.fx * x_world_size / x_pixel_size
            visual_distance_from_y = self.fy * y_world_size / y_pixel_size
            return (visual_distance_from_x + visual_distance_from_y) / 2.0
        return range_dist

    def calculation(self):
        for i in range(len(self.new_data)):
                dist = self.estimate_distance(self.new_data[i])
                left = self.new_data[i].thetaleft
                right = self.new_data[i].thetaright
                if right > left:
                    right -= np.pi * 2
                middle = (left + right)/2.0
                x = dist * np.cos(middle)
                y = dist * np.sin(middle)
                
                (x_global, y_global) = self.local2global(x, y, True)
                #if (i > len(self.new_data) - 1):
                #    return
                name = self.new_name[i]

                if dist < self.thres: 
                        rospy.loginfo("Too Close, Ignore the Value!")
                elif self.avg:
                    self.foodDict_x[name] = x_global
                    self.foodDict_y[name] = y_global
                elif name not in self.foodDict_x:
                    self.foodDict_x[name] = x_global
                    self.foodDict_y[name] = y_global
                    self.foodDict_count[name] = 1
                else:
                    self.foodDict_x[name] = (self.foodDict_x[name] * self.foodDict_count[name] + x_global) / (self.foodDict_count[name] + 1)
                    self.foodDict_y[name] = (self.foodDict_y[name] * self.foodDict_count[name] + y_global) / (self.foodDict_count[name] + 1)
                    self.foodDict_count[name] = self.foodDict_count[name] + 1

    def output(self):
        nameList = []   # a lis of names
        xList = []
        yList = []
        msg = ObjectLocation()

        for key in self.foodDict_x:
                nameList.append(key)
                xList.append(self.foodDict_x[key])
                yList.append(self.foodDict_y[key])

        msg.name = nameList
        msg.x = xList
        msg.y = yList

        self.food_location_pub.publish(msg)


    def local2global(self, x_local, y_local, debug=False):
        # makes sure we have a location
        try:
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            x0 = translation[0]
            y0 = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]

            R = np.array([
                [np.cos(self.theta), -np.sin(self.theta), 0],
                [np.sin(self.theta), np.cos(self.theta), 0],
                [0, 0, 1]
                ])

            if debug:
                rospy.loginfo("*************************************************")
                rospy.loginfo("Angle between two frames: ")
                rospy.loginfo(self.theta)
                rospy.loginfo("Origin in global positon (x, y): ")
                rospy.loginfo(x0)
                rospy.loginfo(y0)
                rospy.loginfo("Local Position (x, y): ")
                rospy.loginfo(x_local)
                rospy.loginfo(y_local)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Error in Frames")
            return (0,0)

        V = np.array([
                [x_local],
                [y_local],
                [0]
                ])

        V_origin = np.array([
                [x0],
                [y0],
                [0]
                ])

        V_global = np.dot(R, V) + V_origin
        if debug:
            rospy.loginfo('x_global is ' + str(V_global[0][0]))
            rospy.loginfo('y_global is ' + str(V_global[1][0]))
            rospy.loginfo("*************************************************")
        return (V_global[0][0], V_global[1][0])

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()



if __name__ == '__main__':
    foods = food()
    foods.run()
