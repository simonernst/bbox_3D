#!/usr/bin/env python2

"""
This program will get a ROS custom message in which we can find each bounding box center's
XYZ coordinates for a detected object and associate a TF to the center coordinates 

Written by Thomas CURE and Simon ERNST
"""
import os
import rospy, roslib
roslib.load_manifest('tf_broadcaster')
import tf
from tf_broadcaster.msg import DetectionCoordinates, PointCoordinates
from geometry_msgs.msg import PointStamped, Pose
import math, json, random
from robocup_msgs.msg import InterestPoint
from rospy_message_converter import message_converter, json_message_converter
from map_manager.srv import *


class ObjectTfBroadcaster(object):

    CONFIG_PATH="/home/simon/catkin_robocup/data/world_mng/interest_points/"

    def __init__(self):
        """
        Create an instance of ObjectTfBroadcaster Class.
        Setup the ROS node and the program's parameters
        """
        rospy.init_node('tfbroadcaster')
        self.br=tf.TransformBroadcaster()
        self.listener=tf.TransformListener()
        self.rate = rospy.Rate(10)
        variable=rospy.get_param('~result_topic')
        self.tf_frame_source=rospy.get_param('~tf_frame_source')
        self.tf_frame_target=rospy.get_param('~tf_frame_target')
        self.sub_detection_object=rospy.Subscriber(variable,DetectionCoordinates,self.handle_message_objects)

    def handle_message_objects(self,req):
        """
        Get the ROS message with the XYZ coordinates and publish a TF with the XYZ point as origin
        """
        print("hello")
        rospy.loginfo("Retrieving interest point directory")
        self.dirs = os.listdir(self.CONFIG_PATH)

        if len(req.points)>0:
            for point in req.points:
                pos_x=point.x
                pos_y=point.y
                pos_z=point.z
                if math.isnan(pos_x)==False and math.isnan(pos_y)==False and math.isnan(pos_z)==False:

                    #Calculating object position in the map frame
                    points=PointStamped()
                    points.header.frame_id=self.tf_frame_source
                    points.header.stamp=rospy.Time(0)
                    points.point.x=point.x
                    points.point.y=point.y
                    points.point.z=point.z
                    p=self.listener.transformPoint("map",points)
                    pos_x=p.point.x
                    pos_y=p.point.y
                    pos_z=p.point.z
                    
                    #Checking if object already saved in Interest Point List
                    count=0
                    c=0
                    for fileName in self.dirs:
                        if not str(point.name) in str(fileName):
                            continue
                        else:
                            c+=1
                            with open(self.CONFIG_PATH + str(fileName)) as json_file:
                                data = json.load(json_file)
                                if pos_x - 0.5 <= data['pose']['position']['x'] <= pos_x + 0.5 and pos_y - 0.5 <= data['pose']['position']['y'] <= pos_y + 0.5 and pos_z - 0.5 <= data['pose']['position']['z'] <= pos_z + 0.5:
                                    rospy.loginfo("Object at the same place - will NOT save as Interest Point")
                                    count+=1
                                    json_file.close()
                                    
                                else:
                                    json_file.close()
                                    continue
                                    
                    if count == 0:
                        rospy.loginfo("Object at a new location - Saving as Interest Point")
                        #save object position as geometry_msgs/Pose
                        itp_pose = Pose()
                        itp_pose.position.x = p.point.x
                        itp_pose.position.y = p.point.y
                        itp_pose.position.z = p.point.z
                        itp_pose.orientation.x = 0
                        itp_pose.orientation.y = 0
                        itp_pose.orientation.z = 0
                        itp_pose.orientation.w = 1

                        #calling MapManager/save_interestPoint Service
                        rospy.wait_for_service('save_InterestPoint')
                        save_InterestPoint = rospy.ServiceProxy('save_InterestPoint', saveitP_service)
                        itPoint=InterestPoint()
                        itPoint.label = "object_" + str(point.name)+str(c)
                        itPoint.pose = itp_pose
                        itPoint.arm_position = 0
                        success = save_InterestPoint(itPoint)
                    #self.listener.waitForTransform(self.tf_frame_target,self.tf_frame_source,rospy.Time(0),rospy.Duration(1.0))

                    #self.br.sendTransform((pos_x,pos_y,pos_z), (0,0,0,1), rospy.Time.now(), tf_name, self.tf_frame_target)

                else:
                    rospy.loginfo("Impossible to calculate depth of object")





if __name__ == '__main__':
    
    a=ObjectTfBroadcaster()
    while not rospy.is_shutdown():
        a.rate.sleep()
