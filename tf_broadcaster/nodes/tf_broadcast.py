#!/usr/bin/env python
__author__ = 'simonernst'
"""
This program will get 3D position of an item from get_coordinate_object node 
and save it as an Interest Point
"""

import rospy
import os, time
import shutil
import math, json, random
from rospy_message_converter import message_converter, json_message_converter
import tf

from tf_broadcaster.msg import DetectionCoordinates, PointCoordinates
from geometry_msgs.msg import PointStamped, Pose
from robocup_msgs.msg import InterestPoint
import thread
from map_manager.srv import *
from tf_broadcaster.srv import CurrentArea, GetObjectsInRoom, GetObjectsInRoomResponse, GetPeopleInRoom, GetPeopleInRoomResponse, ResetObjects, ResetObjectsResponse
from ros_people_mng_msgs.msg import PeopleMetaInfoListWithoutImg



class ObjectTfBroadcaster:

    TEMP_PATH=""
    MAP_MANAGER_PATH=""
    _sleep_time = 2


    def __init__(self,conf_path,conf_path2):
        print(os.getcwd())
        self.TEMP_PATH=conf_path2
        self.MAP_MANAGER_PATH=conf_path
        self.configure()

    def configure(self):
        
        self.dirs = os.listdir(self.TEMP_PATH)
        self.br=tf.TransformBroadcaster()
        self.listener=tf.TransformListener()

        variable=rospy.get_param('~result_topic')
        self.tf_frame_source=rospy.get_param('~tf_frame_source')
        self.tf_frame_target=rospy.get_param('~tf_frame_target')
        self.sub_detection_object=rospy.Subscriber(variable,DetectionCoordinates,self.handle_message_objects)

        self.sub_detection_people = rospy.Subscriber(rospy.get_param("~people_detection_topic"),PeopleMetaInfoListWithoutImg, self.handle_people_detection_topic)

        self.get_objects_list_service = rospy.Service(rospy.get_param("~service_get_objects_in_room"), GetObjectsInRoom, self.handle_service_get_objects)

        self.get_people_list_in_room_service = rospy.Service(rospy.get_param("~service_get_people_in_room"),GetPeopleInRoom,self.handle_service_get_people_in_room)

        self.reset_objects_service = rospy.Service(rospy.get_param("~service_reset_objects"), ResetObjects, self.handle_reset_object_service)
        self.update_score_available = True
        #thread.start_new_thread(self.update_score,())

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def handle_reset_object_service(self,req):
        self.update_score_available = False
        if req.all_objects == True:
            rospy.loginfo("{class_name} : RESET OBJECTS IN TEMP FOLDER".format(class_name=self.__class__.__name__))
            
            for filename in os.listdir(self.TEMP_PATH):
                file_path = os.path.join(self.TEMP_PATH, filename)
                try:
                    if os.path.isfile(file_path) or os.path.islink(file_path):
                        os.unlink(file_path)
                    elif os.path.isdir(file_path):
                        shutil.rmtree(file_path)
                except Exception as e:
                    rospy.logerr('Failed to delete %s. Reason: %s',file_path, e)

            
            rospy.loginfo("{class_name} : RESETTING OBJECT FILES".format(class_name=self.__class__.__name__))
            for filename in os.listdir(self.MAP_MANAGER_PATH):
                if "object_" in filename:
                    file_path = os.path.join(self.MAP_MANAGER_PATH,filename)
                    try:
                        if os.path.isfile(file_path) or os.path.islink(file_path):
                            os.unlink(file_path)
                        elif os.path.isdir(file_path):
                            shutil.rmtree(file_path)
                    except Exception as e:
                        rospy.logerr('Failed to delete %s. Reason: %s',file_path, e)

        else:
            rospy.loginfo("{class_name} : RESETTING OBJECT %s".format(class_name=self.__class__.__name__),req.object_label)
            for filename in os.listdir(self.MAP_MANAGER_PATH):
                if req.object_label in filename:
                    file_path = os.path.join(self.MAP_MANAGER_PATH,filename)
                    try:
                        if os.path.isfile(file_path) or os.path.islink(file_path):
                            os.unlink(file_path)
                        elif os.path.isdir(file_path):
                            shutil.rmtree(file_path)
                    except Exception as e:
                        rospy.logerr('Failed to delete %s. Reason: %s',file_path, e)

            for filename in os.listdir(self.TEMP_PATH):
                if req.object_label in filename:
                    file_path = os.path.join(self.TEMP_PATH,filename)
                    try:
                        if os.path.isfile(file_path) or os.path.islink(file_path):
                            os.unlink(file_path)
                        elif os.path.isdir(file_path):
                            shutil.rmtree(file_path)
                    except Exception as e:
                        rospy.logerr('Failed to delete %s. Reason: %s',file_path, e)
        
        self.update_score_available = True

        #thread.start_new_thread(self.update_score,())

        return ResetObjectsResponse("OK")

    def handle_service_get_objects(self,req):
        room = req.room
        
        list_objects = []

        dirs = os.listdir(self.MAP_MANAGER_PATH)
        for fileName in dirs:
            if "object_"+room in fileName:
                with open(self.MAP_MANAGER_PATH + fileName,"r") as f:
                    data = json.load(f)
                    list_objects.append(json.dumps(data))

        return GetObjectsInRoomResponse(list_objects)


    def handle_service_get_people_in_room(self,req):
        room = req.room

        list_people = []
        dirs = os.listdir(self.MAP_MANAGER_PATH)
        for fileName in dirs:
            if "people_"+room in fileName:
                with open(self.MAP_MANAGER_PATH + fileName,"r") as f:
                    data = json.load(f)
                    list_people.append(json.dumps(data))
                    
        return GetPeopleInRoomResponse(list_people)

    def handle_people_detection_topic(self,req):
        detection_list = req.peopleList
        for detected_people in detection_list:
            if detected_people.label_id == "Unknown" or detected_people.label_id == "None":
                rospy.loginfo("I can't save data from unknown people")

            else:
                people_name = detected_people.label_id
                people_score = detected_people.label_score
                people_position = detected_people.pose.position
                now = rospy.Time(0)
                object_point = PointStamped()
                object_point.header.frame_id = "palbator_arm_kinect_link"
                object_point.header.stamp = now
                object_point.point.x = people_position.x
                object_point.point.y = people_position.y
                object_point.point.z = people_position.z
                rospy.loginfo("{class_name} : Object coords in palbator_arm_kinect_link : %s".format(class_name=self.__class__.__name__),str(object_point))
                self.listener.waitForTransform("/map", "/palbator_arm_kinect_link", now, rospy.Duration(20))
                target = self.listener.transformPoint("/map",object_point)
                rospy.loginfo("{class_name} : Object coords in map : %s".format(class_name=self.__class__.__name__),str(target))

                ####### TO DO : STOCKER LES INFOS DES PERSONNES CONNUES DETECTEES (un peu comme les objets) ######


    def update_score(self):  
        dirs = os.listdir(self.TEMP_PATH)
        itP_dirs = os.listdir(self.MAP_MANAGER_PATH)
        rospy.loginfo("Updating score initiated")
        for fileName in dirs:
            score=0.0
            if "object" in str(fileName):
                json_file = open(self.TEMP_PATH + str(fileName), 'r')
                #rospy.logwarn(fileName)
                data = json.load(json_file)
                json_file.close()
                cumul_darknet = data['confidence_darknet']
                count = data['count']
                overlap = data['overlap']

                #total counts - counts of other objects at the same location
                corrected_counts = count - overlap
                
                #No need for negative values
                if corrected_counts < 0:
                    corrected_counts = 0

                temps = time.time() - data['last_seen']
                round_time=math.ceil(temps)
                val = math.log(round_time+0.0000001)+1
                mean_darknet = cumul_darknet / count

                #Score calculation function
                score = 100*(float(corrected_counts)*0.01*mean_darknet/float(val))/count

                #rospy.loginfo("\n Object label : %s \n Mean confidence : %f \n Time since last seen : %f \n Counts : %d \n Corrected counts : %d", 
                #                str(data['label']), mean_darknet, temps,count,corrected_counts)

                json_tmp = open(self.TEMP_PATH + str(fileName), 'w+')
                data['score']=score
                json_tmp.write(json.dumps(data))
                json_tmp.close()
                #rospy.loginfo("Object %s has a score of %f with %d counts \n", data['label'], score, count)

                if os.path.exists(self.MAP_MANAGER_PATH + str(fileName)):
                    json_itp = open(self.MAP_MANAGER_PATH + str(fileName), 'w+')
                    json_itp.write(json.dumps(data))
                    json_itp.close()
                    #rospy.loginfo("Updating Interest Point %s", fileName)
        #time.sleep(self._sleep_time)
        rospy.loginfo("Update score completed")


    def save_InterestPoint(self):
        tmp_dir = os.listdir(self.TEMP_PATH)
        itP_dirs = os.listdir(self.MAP_MANAGER_PATH)

        for fileName in tmp_dir:
            json_file = open(self.TEMP_PATH + str(fileName), 'r')
            data = json.load(json_file)
            json_file.close()

            if data['score']>0 and data['count']>5 and not os.path.exists(self.MAP_MANAGER_PATH + str(fileName)):
                rospy.loginfo("Saving an object as Interest Point")
                #save object position as geometry_msgs/Pose
                itp_pose = Pose()
                itp_pose.position.x = data['pose']['position']['x']
                itp_pose.position.y = data['pose']['position']['y']
                itp_pose.position.z = data['pose']['position']['z']
                itp_pose.orientation.x = data['pose']['orientation']['x']
                itp_pose.orientation.y = data['pose']['orientation']['y']
                itp_pose.orientation.z = data['pose']['orientation']['z']
                itp_pose.orientation.w = data['pose']['orientation']['w']

                #calling MapManager/save_interestPoint Service
                rospy.wait_for_service('save_InterestPoint')
                save_InterestPoint = rospy.ServiceProxy('save_InterestPoint', saveitP_service)
                itPoint=InterestPoint()
                itPoint.count = data['count']
                itPoint.confidence_darknet = data['confidence_darknet']
                itPoint.last_seen = data['last_seen']
                itPoint.label = data['label']
                itPoint.pose = itp_pose
                itPoint.arm_position = 0
                success = save_InterestPoint(itPoint)
            
            #purging Interest Points too old and scoreless
            elif (time.time() - data['last_seen']) > 200 and data['score']==0 and os.path.exists(self.MAP_MANAGER_PATH + str(fileName)):
                file_remove=str(self.MAP_MANAGER_PATH)+str(fileName)
                rospy.loginfo("Removing file %s", file_remove)
                os.remove(file_remove)
                itP_dirs = os.listdir(self.MAP_MANAGER_PATH)
                
            elif (time.time() - data['last_seen']) > 10000 and os.path.exists(self.MAP_MANAGER_PATH + str(fileName)):
                file_remove=str(self.MAP_MANAGER_PATH)+str(fileName)
                rospy.loginfo("Removing file %s", file_remove)
                os.remove(file_remove)
                itP_dirs = os.listdir(self.MAP_MANAGER_PATH)


    def handle_message_objects(self,req):
        """
        Get the ROS message with the XYZ coordinates and publish a TF with the XYZ point as origin
        """
        tic=time.time()
        self.dirs = os.listdir(self.TEMP_PATH)
        #os.system('clear')
        #rospy.loginfo("Reading interest point directory \n")

        if len(req.points)>0:
            for point in req.points:
                pos_x = point.x
                pos_y = point.y
                pos_z = point.z
                darknet_score = point.score
                print(pos_x, pos_y, pos_z)
                #Check if 3D pose available
                if math.isnan(pos_x)==False and math.isnan(pos_y)==False and math.isnan(pos_z)==False:

                    #Calculating object position in the map frame
                    points = PointStamped()
                    points.header.frame_id = self.tf_frame_source
                    points.header.stamp=rospy.Time(0)
                    points.point.x = point.x
                    points.point.y = point.y
                    points.point.z = point.z
                    self.listener.waitForTransform("map",self.tf_frame_source, rospy.Time(0), rospy.Duration(5.0))
                    p = self.listener.transformPoint("map",points)
                    pos_x = p.point.x
                    pos_y = p.point.y
                    pos_z = p.point.z
                        
                    #Refactoring checking if object already in Itp list 
                    count=0
                    c=0

                    for fileName in self.dirs:
                        if "object" in str(fileName):
                            with open(self.TEMP_PATH + str(fileName), 'r') as json_file:
                                #rospy.logwarn(fileName)
                                data = json.load(json_file)
                                json_file.close()
                            if pos_x - 0.5 <= data['pose']['position']['x'] <= pos_x + 0.5 and pos_y - 0.5 <= data['pose']['position']['y'] <= pos_y + 0.5 and pos_z - 0.5 <= data['pose']['position']['z'] <= pos_z + 0.5:
                                #Object detected at the same spot
                                if str(point.name) in str(fileName):
                                    c+=1
                                    #same object -> updating last seen / increase count / darknet confidence
                                    count += 1
                                    json_new = open(self.TEMP_PATH + str(fileName), 'w+')

                                    data['count'] += 1
                                    data['confidence_darknet'] += darknet_score
                                    data['last_seen'] = time.time()

                                    json_new.write(json.dumps(data))
                                    json_new.close()

                                else:
                                    #diff object at the same place -> decreasing count and adding overlap info
                                    if data['overlap'] < data['count']:
                                        data['overlap'] += 1
                                        json_new = open(self.TEMP_PATH + str(fileName), 'w+')
                                        json_new.write(json.dumps(data))
                                        json_new.close()                 

                            else:
                                if str(point.name) in str(fileName):
                                    c+=1
                                #object at a different place : in case it has never been seen it will be saved after
                                continue



                    #Saving new object in Itp list
                    if count == 0:

                    #Saving to temp dir

                        rospy.loginfo("Object at a new location - Saving information")
                        #save object position as geometry_msgs/Pose
                        itp_pose = Pose()
                        itp_pose.position.x = p.point.x
                        itp_pose.position.y = p.point.y
                        itp_pose.position.z = p.point.z
                        itp_pose.orientation.x = 0
                        itp_pose.orientation.y = 0
                        itp_pose.orientation.z = 0
                        itp_pose.orientation.w = 1

                        #calling tf_broadcaster/getCurrentArea Service
                        service_name = rospy.get_param("~service_get_current_area_name")
                        rospy.wait_for_service(service_name)
                        self.proxy_areas = rospy.ServiceProxy(service_name, CurrentArea)
                        response = self.proxy_areas(itp_pose.position.x,itp_pose.position.y)

                        if response.room != '':
                            current_room = response.room
                        else:
                            current_room = ''

                        itPoint=InterestPoint()
                        itPoint.count = 1
                        itPoint.confidence_darknet = darknet_score
                        itPoint.last_seen = time.time()
                        itPoint.label = "object_" + current_room + "_" + str(point.name)+str(c)
                        itPoint.pose = itp_pose
                        itPoint.arm_position = 0
                        #success = save_InterestPoint(itPoint)


                        temp_file = open(self.TEMP_PATH + str(itPoint.label) + '.coord', 'w+')
                        json_str = json_message_converter.convert_ros_message_to_json(itPoint)
                        temp_file.write(json_str)
                        temp_file.close()
                    #self.listener.waitForTransform(self.tf_frame_target,self.tf_frame_source,rospy.Time(0),rospy.Duration(1.0))

                    #self.br.sendTransform((pos_x,pos_y,pos_z), (0,0,0,1), rospy.Time.now(), tf_name, self.tf_frame_target)

                else:
                    rospy.loginfo("Impossible to calculate depth of object")
        self.save_InterestPoint()
        self.update_score()
        tac=time.time()
        process=tac-tic
        rospy.loginfo("Process time %f  ",process)

        


if __name__ == '__main__':
    
    rospy.init_node('tfbroadcaster')
    
    try:
        map_value="../../../../data/world_mng/interest_points/"
        temp_value="../../../../data/world_mng/temp/"
    except:
        pass
        #Find a way to create those directories if non existent 


    config_directory_param=rospy.get_param("~confPath",map_value)
    config_directory_param2=rospy.get_param("~trucpath",temp_value)

    mm = ObjectTfBroadcaster(config_directory_param,config_directory_param2)
