#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from using_markers.cfg import generalconfigConfig

def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {int_param}, {wv_param},\ 
    #       {wp_param}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("general_config", anonymous = False)

    srv = Server(generalconfigConfig, callback)
    rospy.spin()