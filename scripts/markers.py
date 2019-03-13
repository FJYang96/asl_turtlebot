#!/usr/bin/env python

import rospy
import math
import numpy as np
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Pose2D, PoseStamped
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from asl_turtlebot.msg import DetectedObjectList
from std_msgs.msg import Header, Float32MultiArray

mapping = rospy.get_param("map")

class MultipleMarkers:
    
    def __init__(self):
        rospy.init_node('tb3_visualizer')
        while rospy.Time().now() == rospy.Time(0):
            pass

        self.x_g, self.y_g, self.theta_g = None, None, None
        self.x, self.y, self.theta = None, None, None
        self.cx, self.cy, self.fx, self.fy = 1., 1., 1., 1.
        self.ymin, self.ymax, self.xmin, self.xmax = 1., 1., 1., 1.
        self.goal_pose_received = False
        self.puddle_detected = False
        self.obj_detected = False
        self.image_plane_dis = 1.0
        self.rviz_listener = tf.TransformListener()
        self.footprint_listener = tf.TransformListener()
        self.down_left, self.down_right, self.up_left, self.up_right = None, None, None, None

        self.tb3_pub = rospy.Publisher('visualization_marker', Marker, queue_size = 10)
        self.fov_pub = rospy.Publisher('camera_FOV', Marker, queue_size = 10)
        self.pose_goal_pub = rospy.Publisher('visualized_des_pose', Marker, queue_size = 10)
        self.detect_obj_pub = rospy.Publisher('detect_obj_FOV', Marker, queue_size = 10)
#        self.detect_puddle_pub = rospy.Publisher('detected_puddle', Marker, queue_size = 10)

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        rospy.Subscriber('/camera_relay/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.detect_obj_callback)
#        rospy.Subscriber('/puddle_center', Float32MultiArray, self.puddle_callback)

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        rospy.loginfo("rviz command received!")
        try:
            origin_frame = "/map" if mapping else "/odom"
            rospy.loginfo("getting frame")
            nav_pose_origin = self.rviz_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
            self.goal_pose_received = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def camera_info_callback(self, msg):
        rospy.loginfo("camera_info_callback")
        self.cx = msg.P[2]
        self.cy = msg.P[6]
        self.fx = msg.P[0]
        self.fy = msg.P[5]

    def detect_obj_callback(self, msg):
        if len(msg.ob_msgs) > 0:
            [ymin, xmin, ymax, xmax] = msg.ob_msgs[-1].corners
            self.down_left = self.project_pixel_to_ray(xmin, ymin)
            self.down_right = self.project_pixel_to_ray(xmax, ymin)
            self.up_left = self.project_pixel_to_ray(xmin, ymax)
            self.up_right = self.project_pixel_to_ray(xmax, ymax)
            self.obj_detected = True

#            rospy.loginfo(self.down_left)
#            rospy.loginfo(self.down_right)
#            rospy.loginfo(self.up_left)
#            rospy.loginfo(self.up_right)
#            rospy.loginfo('************************')

#    def puddle_callback(self, msg):
#        self.puddle_ct_x, self.puddle_ct_y = msg.data
#        self.puddle_detected = True

    def project_pixel_to_ray(self,u,v):
        x = (u - self.cx)/self.fx
        y = (v - self.cy)/self.fy
        norm = math.sqrt(x*x + y*y + 1)
#        rospy.loginfo(u)
#        rospy.loginfo(self.cx)
#        rospy.loginfo(self.fx)
#        rospy.loginfo('************************')
        x /= norm
        y /= norm
        z = 1.0 / norm

        y, z = -self.image_plane_dis * x, 0.09 - self.image_plane_dis * y

        return (y,z)

    def loop(self):
        
        # Turtlebot3 Marker
        tb3_marker = Marker()
        tb3_marker.header.frame_id = "/base_footprint"
        tb3_marker.header.stamp = rospy.Time(0)
        tb3_marker.ns = "tb3_cylinder"
        tb3_marker.action = tb3_marker.ADD
        tb3_marker.type = tb3_marker.CYLINDER
        tb3_marker.scale.x = 0.15
        tb3_marker.scale.y = 0.15
        tb3_marker.scale.z = 0.15
        tb3_marker.color.a = 1.0
        tb3_marker.color.b = 1.0
        tb3_marker.frame_locked = False
        tb3_marker.lifetime = rospy.Duration()
        self.tb3_pub.publish(tb3_marker)

        # Camera FOV
        fov_marker = Marker()
        fov_marker.header.frame_id = "/base_footprint"
        fov_marker.header.stamp = rospy.Time(0)
        fov_marker.ns = "raspicam_FOV"
        fov_marker.action = fov_marker.ADD
        fov_marker.type = fov_marker.LINE_LIST
        fov_marker.scale.x = 0.01
        fov_marker.color.g = 1.0
        fov_marker.color.a = 1.0

        # set of points

        p1, p2, p3, p4, p5 = Point(), Point(), Point(), Point(), Point()
        p1.x, p1.y, p1.z = 0., 0., .09
        p2.x, p2.y, p2.z = 1.5, 0.9, 0.
        p3.x, p3.y, p3.z = 1.5, -0.9, 0.
        p4.x, p4.y, p4.z = 1.5, 0.9, 0.77
        p5.x, p5.y, p5.z = 1.5, -0.9, 0.77

        fov_marker.points.append(p1)
        fov_marker.points.append(p2)
        fov_marker.points.append(p1)
        fov_marker.points.append(p3)
        fov_marker.points.append(p1)
        fov_marker.points.append(p4)
        fov_marker.points.append(p1)
        fov_marker.points.append(p5)
        fov_marker.points.append(p2)
        fov_marker.points.append(p3)
        fov_marker.points.append(p2)
        fov_marker.points.append(p4)
        fov_marker.points.append(p3)
        fov_marker.points.append(p5)
        fov_marker.points.append(p4)
        fov_marker.points.append(p5)

        fov_marker.frame_locked = True
        fov_marker.lifetime = rospy.Duration()
        self.fov_pub.publish(fov_marker)

        # Desired Pose
        if self.goal_pose_received:
            des_pose_markers = Marker()
            des_pose_markers.header.frame_id = "/map"
            des_pose_markers.header.stamp = rospy.Time.now()
            des_pose_markers.ns = "desired_pose_marker"
            des_pose_markers.action = des_pose_markers.ADD
            des_pose_markers.type = des_pose_markers.ARROW
            des_pose_markers.pose.orientation.w = np.cos(self.theta_g / 2.)
            des_pose_markers.pose.orientation.z = np.sin(self.theta_g / 2.)
            des_pose_markers.pose.position.x = self.x_g
            des_pose_markers.pose.position.y = self.y_g
            des_pose_markers.pose.position.z = 0.0
            des_pose_markers.scale.x = 0.2
            des_pose_markers.scale.y = 0.1
            des_pose_markers.scale.z = 0.05
            des_pose_markers.color.a = 1.0
            des_pose_markers.color.r = 1.0
            des_pose_markers.color.g = 0.0
            des_pose_markers.color.b = 0.0
            des_pose_markers.lifetime = rospy.Duration(1000)  
            self.pose_goal_pub.publish(des_pose_markers)

        # Detected object
        if self.obj_detected:
            self.obj_detected = False
            obj_marker = Marker()
            obj_marker.header.frame_id = "/base_footprint"
            obj_marker.header.stamp = rospy.Time(0)
            obj_marker.ns = "detected_obj"
            obj_marker.action = fov_marker.ADD
            obj_marker.type = fov_marker.LINE_LIST
            obj_marker.scale.x = 0.005
            obj_marker.color.g = 1.0
            obj_marker.color.a = 1.0

            # set of points
            op1, op2, op3, op4, op5, op6, op7, op8 = Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()
            op9, op10, op11, op12 = Point(), Point(), Point(), Point()
            op13, op14, op15, op16 = Point(), Point(), Point(), Point()
            op1.x, op1.y, op1.z = 0., 0., .09
            op2.x, op2.y, op2.z = 1.5, self.down_left[0], self.down_left[1]
            op3.x, op3.y, op3.z = 0., 0., 0.09
            op4.x, op4.y, op4.z = 1.5, self.down_right[0], self.down_right[1]
            op5.x, op5.y, op5.z = 0., 0., 0.09
            op6.x, op6.y, op6.z = 1.5, self.up_left[0], self.up_left[1]
            op7.x, op7.y, op7.z = 0., 0., 0.09
            op8.x, op8.y, op8.z = 1.5, self.up_right[0], self.up_right[1]

            op9.x, op9.y, op9.z = 1.5, self.down_left[0], self.down_left[1]
            op10.x, op10.y, op10.z = 1.5, self.down_right[0], self.down_right[1]
            op11.x, op11.y, op11.z = 1.5, self.down_right[0], self.down_right[1]
            op12.x, op12.y, op12.z = 1.5, self.up_right[0], self.up_right[1]
            op13.x, op13.y, op13.z = 1.5, self.up_right[0], self.up_right[1]
            op14.x, op14.y, op14.z = 1.5, self.up_left[0], self.up_left[1]
            op15.x, op15.y, op15.z = 1.5, self.up_left[0], self.up_left[1]
            op16.x, op16.y, op16.z = 1.5, self.down_left[0], self.down_left[1]

            obj_marker.points.append(op1)
            obj_marker.points.append(op2)
            obj_marker.points.append(op3)
            obj_marker.points.append(op4)
            obj_marker.points.append(op5)
            obj_marker.points.append(op6)
            obj_marker.points.append(op7)
            obj_marker.points.append(op8)
            obj_marker.points.append(op9)
            obj_marker.points.append(op10)
            obj_marker.points.append(op11)
            obj_marker.points.append(op12)
            obj_marker.points.append(op13)
            obj_marker.points.append(op14)
            obj_marker.points.append(op15)
            obj_marker.points.append(op16)
            obj_marker.lifetime = rospy.Duration(100)

            self.detect_obj_pub.publish(obj_marker)

#        # Puddle Marker
#        if self.puddle_detected:
#            puddle_marker = Marker()
#            puddle_marker.header.frame_id = "/map"
#            puddle_marker.header.stamp = rospy.Time.now()
#            puddle_marker.ns = "puddle"
#            puddle_marker.action = puddle_marker.ADD
#            puddle_marker.type = puddle_marker.CYLINDER
#            puddle_marker.pose.orientation.w = 1.0
#            puddle_marker.scale.x = 0.2
#            puddle_marker.scale.y = 0.2
#            puddle_marker.scale.z = 0.005
#            puddle_marker.color.a = 1.0
#            puddle_marker.color.r = 1.0
#            puddle_marker.color.g = 0.0
#            puddle_marker.color.b = 1.0
#            puddle_marker.pose.position.x = self.puddle_ct_x
#            puddle_marker.pose.position.y = self.puddle_ct_y
#            puddle_marker.lifetime = rospy.Duration(10000)
#            self.detect_puddle_pub.publish(puddle_marker)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = MultipleMarkers()
    sup.run()
