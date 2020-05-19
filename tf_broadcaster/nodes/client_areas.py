#!/usr/bin/env python

import rospy
from tf_broadcaster.srv import CurrentArea

class Test:

    def __init__(self):

        rospy.init_node("client_areas")

        self.proxy_areas = rospy.ServiceProxy("get_current_area_in_map",CurrentArea)

        response = self.proxy_areas(3.5,2.5)

        rospy.loginfo("ROOM : %s",str(response.room))

if __name__ == "__main__":
    a=Test()