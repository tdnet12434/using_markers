#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from using_markers.cfg import pidConfig

def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {wv_param}, {wp_param},\ 
    #       {wbias_param}, {wv_param}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("PID_TUNE", anonymous = False)

    srv = Server(pidConfig, callback)
    rospy.spin()