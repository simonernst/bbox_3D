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

        with open(dir_path+"/../data/map_ycb_areas.json","r") as f:
                self.areas=json.load(f)
        
        rospy.loginfo(self.areas)
        self.current_area_service = rospy.Service('get_current_area_in_map', CurrentArea, self.handle_current_area_service)

        rospy.loginfo("init done")


    def handle_current_area_service(self,req):

        x_robot = req.coord_x
        y_robot = req.coord_y
        for zone in self.areas:
            if x_robot > zone['x_lowerlimit'] and x_robot < zone['x_upperlimit'] and y_robot > zone['y_lowerlimit'] and y_robot < zone['y_upperlimit']:
                return CurrentAreaResponse(str(zone['label']))

        return CurrentAreaResponse("Unable to find your location right now")

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

