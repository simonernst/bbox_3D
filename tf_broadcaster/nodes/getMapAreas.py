#!/usr/bin/env python

import rospy
import json
from tf import TransformListener
from std_msgs.msg import String

from tf_broadcaster.srv import CurrentArea, CurrentAreaResponse
import os

class MapYcbAreas:

    def __init__(self):
        rospy.init_node("YCB_areas")
        dir_path = os.path.dirname(os.path.realpath(__file__))

        with open(dir_path+rospy.get_param("~map_file_path"),"r") as f:
                self.areas=json.load(f)
        
        rospy.loginfo(self.areas)
        self.current_area_service = rospy.Service(rospy.get_param("~service_get_current_area_name"), CurrentArea, self.handle_current_area_service)

        rospy.loginfo("init done")


    def handle_current_area_service(self,req):
        rospy.logwarn("get map area service")
        x_robot = req.coord_x
        y_robot = req.coord_y
        rospy.loginfo("X Robot : %s",str(x_robot))
        rospy.loginfo("Y Robot : %s",str(y_robot))
        for zone in self.areas:
            rospy.loginfo("area name : %s",zone['label'])
            rospy.loginfo("area X min : %s",str(zone['x_lowerlimit']))
            rospy.loginfo("area X max : %s",str(zone['x_upperlimit']))
            rospy.loginfo("area Y min : %s",str(zone['y_lowerlimit']))
            rospy.loginfo("area Y max : %s",str(zone['y_upperlimit']))
            if x_robot > zone['x_lowerlimit'] and x_robot < zone['x_upperlimit'] and y_robot > zone['y_lowerlimit'] and y_robot < zone['y_upperlimit']:
                return CurrentAreaResponse(str(zone['label']))

        return CurrentAreaResponse('')

    # def get_current_pos(self):
    #     now = rospy.Time(0)
    #     self.listener.waitForTransform("/map", "base_footprint", now, rospy.Duration(2))
    #     (trans, rot) = self.listener.lookupTransform("/map", "base_footprint", now)
        
    #     x_robot = trans[0]
    #     y_robot = trans[1]

    #     for zone in self.areas:
    #         if x_robot > zone['x_lowerlimit'] and x_robot < zone['x_upperlimit'] and y_robot > zone['y_lowerlimit'] and y_robot < zone['y_upperlimit']:
    #             rospy.logwarn("current zone: %s",str(zone['label']))
    #             break


if __name__ == "__main__":
    x = MapYcbAreas()

    while not rospy.is_shutdown():
        # x.get_current_pos()
        rospy.spin()

