#!/usr/bin/env python

PACKAGE = "using_markers"
import roslib;roslib.load_manifest(PACKAGE)
import rospy

import dynamic_reconfigure.client



def callback(config):
    rospy.loginfo("gg")
    # rospy.loginfo("""Reconfigure Request: {wv_param}, {wp_param},\ 
    #       {wbias_param}, {wv_param}, {delaygps_param}""".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("general_config", timeout=5, config_callback=callback)

    r = rospy.Rate(1)
    # x = 0
    b = False
    while not rospy.is_shutdown():
        # x = x+1
        # if x>4:
        #     x=0
        #b = not b
        client.update_configuration({"testnumsen_param":b})
        r.sleep()