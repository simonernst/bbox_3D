#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, PointStamped, Polygon, PolygonStamped
from std_msgs.msg import Header
from tf_broadcaster.srv import BuildingMapArea, BuildingMapAreaResponse, SavingMapArea, SavingMapAreaResponse
import json
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import os
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from tf_broadcaster.srv import CurrentArea, CurrentAreaResponse

class MapBuilder():

    def __init__(self):
        rospy.init_node("map_builder")


        self.map_building_service = rospy.Service(rospy.get_param("~service_building_map_area"),BuildingMapArea,self.handle_building_map_area)
        rospy.loginfo("{class_name} : service building map area connected".format(class_name=self.__class__.__name__))
        
        self.map_saving_service = rospy.Service(rospy.get_param("~service_saving_map_area"),SavingMapArea,self.handle_saving_map_area)
        rospy.loginfo("{class_name} : service saving map area connected".format(class_name=self.__class__.__name__))

        

        self.server = InteractiveMarkerServer("simple_marker")

        self.coord_IMs = {}
        self.current_dir = os.path.dirname(os.path.realpath(__file__))


        self.load_last_map()


        self.current_area_service = rospy.Service(rospy.get_param("~service_get_current_area_name"), CurrentArea, self.handle_current_area_service)
        rospy.loginfo("{class_name} : service get current map area connected".format(class_name=self.__class__.__name__))

        rospy.loginfo("{class_name} : Init completed".format(class_name=self.__class__.__name__))


    def handle_current_area_service(self,req):
        x_robot = req.coord_x
        y_robot = req.coord_y
        point_to_check = Point(x_robot, y_robot)

        for room in self.coord_IMs.keys():
            list_points = []
            for i in range(0,len(self.coord_IMs[room].keys())):
                list_points.append((self.coord_IMs[room][str(i)]['x'],self.coord_IMs[room][str(i)]['y']))
            
            self.polygon_room = Polygon(list_points)
            validation = self.polygon_room.contains(point_to_check)
            if validation == True:
                return CurrentAreaResponse(room)
        
        return CurrentAreaResponse('')

    def load_last_map(self):

        rospy.loginfo(self.current_dir)
        file_path = os.path.join(self.current_dir,rospy.get_param("~map_file_path"))

        try:
            with open(file_path,'r+') as json_file:
                data = json.load(json_file)
                
            self.coord_IMs = data


            for room in data.keys():
                for point_id in data[str(room)].keys():
                    int_marker = InteractiveMarker()
                    int_marker.header.frame_id = "/map"
                    int_marker.name = "marker_" + room + "_" + point_id
                    int_marker.description = "marker_" + room + "_" + point_id

                    int_marker.pose.position.x = data[str(room)][point_id]['x']
                    int_marker.pose.position.y = data[str(room)][point_id]['y']
                    # create a grey box marker
                    box_marker = Marker()
                    box_marker.header.frame_id = "map"
                    box_marker.type = Marker.CUBE
                    box_marker.pose.position.x = data[str(room)][point_id]['x']
                    box_marker.pose.position.y = data[str(room)][point_id]['y']
                    box_marker.scale.x = 0.2
                    box_marker.scale.y = 0.2
                    box_marker.scale.z = 0.2
                    box_marker.color.r = 0.0
                    box_marker.color.g = 0.5
                    box_marker.color.b = 0.5
                    box_marker.color.a = 1.0

                    # create a non-interactive control which contains the box
                    box_control = InteractiveMarkerControl()
                    box_control.always_visible = True
                    box_control.markers.append( box_marker )

                    # add the control to the interactive marker
                    int_marker.controls.append( box_control )

                    # create a control which will move the box
                    # this control does not contain any markers,
                    # which will cause RViz to insert two arrows


                    rotate_control = InteractiveMarkerControl()
                    rotate_control.orientation.w = 1
                    rotate_control.orientation.x = 1
                    rotate_control.orientation.y = 0
                    rotate_control.orientation.z = 0
                    rotate_control.name = "move_x"
                    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
                    

                    # # add the control to the interactive marker
                    int_marker.controls.append(rotate_control)

                    rotate_control = InteractiveMarkerControl()
                    rotate_control.orientation.w = 1
                    rotate_control.orientation.x = 0
                    rotate_control.orientation.y = 0
                    rotate_control.orientation.z = 1
                    rotate_control.name = "move_y"
                    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
                    int_marker.controls.append(rotate_control)
                    # add the interactive marker to our collection &
                    # tell the server to call processFeedback() when feedback arrives for it
                    self.server.insert(int_marker, self.processFeedback)

                    self.server.applyChanges()

            rospy.loginfo("{class_name} : Map loaded".format(class_name=self.__class__.__name__))
        except:
            rospy.logwarn("{class_name} : No map could be loaded".format(class_name=self.__class__.__name__))


    def handle_saving_map_area(self,req):

        file_path = os.path.join(self.current_dir,rospy.get_param("~map_file_path"))

        try:
            with open(file_path,'r+') as json_file:
                data = json.load(json_file)
        except Exception as e:
            rospy.logwarn(e)
            data = {}

        if data == {}:
            with open(file_path,'w+') as json_file:
                json.dump(self.coord_IMs, json_file, indent=4)
                json_file.truncate()

        else:
            for key in self.coord_IMs.keys():
                for id in self.coord_IMs[key].keys():
                    data[key][id]=self.coord_IMs[key][id]

            with open(file_path,'w+') as json_file:
                json.dump(data, json_file, indent=4)
                json_file.truncate()

        return SavingMapAreaResponse('OK')

        

    def processFeedback(self,feedback):
        p = feedback.pose.position
        rospy.loginfo("{class_name} : ".format(class_name=self.__class__.__name__) + feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

        room = feedback.marker_name.split("_")[1]

        self.coord_IMs[room][feedback.marker_name.split("_")[-1]] = {
            "x": p.x,
            "y": p.y
        }
        rospy.loginfo("-------")

        rospy.loginfo(self.coord_IMs)
        rospy.loginfo("-------")


    def handle_building_map_area(self,req):
        area_name = req.area_name
        interactive_markers_number = req.interactive_markers_number
        self.coord_IMs[area_name]={}

        for i in range (0,interactive_markers_number):
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "/map"
            int_marker.name = "marker_" + area_name + "_" + str(i)
            int_marker.description = "marker_" + area_name + "_" + str(i)

            # create a grey box marker
            box_marker = Marker()
            box_marker.type = Marker.CUBE
            box_marker.scale.x = 0.2
            box_marker.scale.y = 0.2
            box_marker.scale.z = 0.2
            box_marker.color.r = 0.0
            box_marker.color.g = 0.5
            box_marker.color.b = 0.5
            box_marker.color.a = 1.0

            # create a non-interactive control which contains the box
            box_control = InteractiveMarkerControl()
            box_control.always_visible = True
            box_control.markers.append( box_marker )

            # add the control to the interactive marker
            int_marker.controls.append( box_control )

            # create a control which will move the box
            # this control does not contain any markers,
            # which will cause RViz to insert two arrows


            rotate_control = InteractiveMarkerControl()
            rotate_control.orientation.w = 1
            rotate_control.orientation.x = 1
            rotate_control.orientation.y = 0
            rotate_control.orientation.z = 0
            rotate_control.name = "move_x"
            rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            

            # # add the control to the interactive marker
            int_marker.controls.append(rotate_control)

            rotate_control = InteractiveMarkerControl()
            rotate_control.orientation.w = 1
            rotate_control.orientation.x = 0
            rotate_control.orientation.y = 0
            rotate_control.orientation.z = 1
            rotate_control.name = "move_y"
            rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(rotate_control)
            # add the interactive marker to our collection &
            # tell the server to call processFeedback() when feedback arrives for it
            self.server.insert(int_marker, self.processFeedback)

            self.coord_IMs[area_name][str(i)]= {
                "x": 0,
                "y": 0
            }

            # 'commit' changes and send to all clients
            self.server.applyChanges()


        return BuildingMapAreaResponse('OK')
        
if __name__ == "__main__":
    map_builder = MapBuilder()
    while not rospy.is_shutdown():
        # if map_builder.polygon_stamped:
        #     map_builder.pub_polygon.publish(map_builder.polygon_stamped)
        rospy.spin()