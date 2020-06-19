#!/usr/bin/env python

import os, sys
import numpy as np
import cv2
import math
import statistics
from itertools import compress

import rospy
import ros_numpy as rosnp
import tf

import std_msgs.msg

from geometry_msgs.msg import Point32

from yolact_ros_msgs.msg import Detections
from yolact_ros_msgs.msg import Detection
from yolact_ros_msgs.msg import Box
from yolact_ros_msgs.msg import Mask
from yolact_ros_msgs.msg import Mask_Depth, Mask_Coordinates


import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros


#Debug printing whole array
np.set_printoptions(threshold=sys.maxsize)


#USE CASE a prendre en compte
#Detection pendant robot en mouvement ?

class MaskTo3D(object):
    
    nok_class = ['person','car','bycicle','motorcycle','airplane','bus','train','truck','boat','traffic light','fire hydrant'
                'stop sign','parking meter','sheep','cow','elephant','bear','zebra','giraffe','sheep','horse']
    ok_class = []

    # TODO : ADD THOSE PARAMS IN CONFIG FILE

    #config for real robot
    #depth_topic="/kinect2/hd/points"
    #frame_source="kinect2_rgb_optical_frame"


    #config for Palbator simulation
    frame_source = "kinect2_rgb_optical_frame"
    depth_topic  = "/kinect/depth/points"

    def __init__(self):
        rospy.init_node("depth_mask",anonymous=True)
        mask_msg="/yolact_ros/detections"
        self.listener=tf.TransformListener()
        self.pointcloud_sub=rospy.Subscriber("/kinect/depth/points", PointCloud2, self.pcl_callback)
        self.sub_mask_msg=rospy.Subscriber(mask_msg,Detections,self.mask_msg_callback)
        self.pub_mask_depth=rospy.Publisher("/mask_coordinates", Mask_Depth, queue_size=1)
        self.cloud_msg=PointCloud2()
        #self.pointcloud_publisher = rospy.Publisher("/my_pointcloud_topic", Pointcloud)
        self.rate=rospy.Rate(100)
        self.br = tf.TransformBroadcaster()
        self.done = 1
        self.valid=True

    def pcl_callback(self,req):
        #get the data of the PointCloud from RGBD camera
        self.cloud_msg=req

    def mask_msg_callback(self,req):

        array=rosnp.point_cloud2.pointcloud2_to_xyz_array(self.cloud_msg,False)

        label_store,score_store,box_store,mask_store =[],[],[],[]

        #custom msg creation with 3D coordinates of each pixel in each masks
        #message=Mask_Depth()
        #message.header.stamp=rospy.Time.now()

        rospy.loginfo("Parsing detections")
        for item in req.detections:

            #Init variables
            points_list = []
            #Get mask array and starting coordinate of the bounding box
            obj_mask=item.mask.mask
            x_offset=item.box.x1
            y_offset=item.box.y1
            
            #create (X,Y) coordinates of a rectagle with the same size of the bounding box
            img = np.full((item.mask.height,item.mask.width,3), (0,0,255), dtype=np.uint8)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            xy_coords = np.flip(np.column_stack(np.where(gray >= 0)), axis=1)
            
            #adding an offset to fit the coordinates of the actual bounding box of the mask
            for pixels in xy_coords:
                pixels[0]+=x_offset
                pixels[1]+=y_offset

            #checking only coordinates with mask inside the bounding box
            coord_mask = list(compress(xy_coords, obj_mask))
            rospy.loginfo("AAA")

            """
            create custom message with 3D points of mask
            mask_message = Mask_Coordinates()
            mask_message.class_name = label_store[i]
            mask_message.score = score_store[i] 
            """

            # Parsing each pixel of the mask, add the Z coordinate from Pointcloud and 
            # transform the mask into the map frame
            for j in range(0,len(coord_mask)):

                if math.isnan(array[coord_mask[j][1]][coord_mask[j][0]][0]) or \
                    math.isnan(array[coord_mask[j][1]][coord_mask[j][0]][1]) or \
                    math.isnan(array[coord_mask[j][1]][coord_mask[j][0]][2]):
                    pass
                else:
                    #points = PointStamped()
                    #points.header.frame_id = "kinect_depth_optical_frame"
                    #points.header.frame_id = "kinect_depth_link"
                    #points.header.stamp=rospy.Time(0)
                    #points.point.x = array[coord_mask[j][1]][coord_mask[j][0]][0]
                    #points.point.y = array[coord_mask[j][1]][coord_mask[j][0]][1]
                    #points.point.z = array[coord_mask[j][1]][coord_mask[j][0]][2]
                    #p = self.listener.transformPoint("map",points)
                    points_list.append([array[coord_mask[j][1]][coord_mask[j][0]][0],
                                        array[coord_mask[j][1]][coord_mask[j][0]][1],
                                        array[coord_mask[j][1]][coord_mask[j][0]][2]])


            #Create a PCL in the camera link        
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id= 'kinect_depth_link'   #change to ros.getparam
            scaled_polygon_pcl = pcl2.create_cloud_xyz32(h, points_list)


            #Transform the PCL into the map frame
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            transform = tf_buffer.lookup_transform("map","kinect_depth_link", rospy.Time())
            cloud_out = do_transform_cloud(scaled_polygon_pcl, transform)

            #self.listener.waitForTransform("kinect_depth_link","map", rospy.Time(0), rospy.Duration(1.0))
            #p=self.listener.transformPointCloud("map", scaled_polygon_pcl.)


            

            rospy.loginfo("BBB")
            if self.valid:
                rospy.loginfo("valid")
                list_tot=[]
                for l in range(0,len(list_x)):
                    list_tot.append([list_x[l],list_y[l],list_z[l]])

                with open('list_tot.txt', 'w') as f:
                    for stuff in list_tot:
                        f.write("%s\n" % stuff)

                for k in range(0,len(list_x)):
                    tf_name=str(item.class_name)+str(k)
                    #print(tf_name)
                    self.br.sendTransform((list_x[k],list_y[k],list_z[k]),
                                (0.0, 0.0, 0.0, 1.0),
                                rospy.Time(0),
                                tf_name,
                                "map")
                self.valid = False  
            #mask_message.xcoord = list_x
            #mask_message.ycoord = list_y
            #mask_message.zcoord = list_z
            #message.coordinates.append(mask_message)
        #self.pub_mask_depth.publish(message)

if __name__=="__main__":
    a=MaskTo3D()
    while not rospy.is_shutdown():
        a.rate.sleep()

