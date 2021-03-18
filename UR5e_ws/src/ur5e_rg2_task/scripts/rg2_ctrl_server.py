#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import print_function #_future_ must be put at the beginning

NAME = 'rg2_control_server'

import rg2_gripper
from ur5e_rg2_task.srv import rg2_control, rg2_controlResponse
import rospy

def handle_rg2_control(req):
    rg2_gripper.setGripperWidth(req.width, req.force)
    return rg2_controlResponse(True)

def rg2_control_server():
    rospy.init_node(NAME)
    s = rospy.Service('rg2_control', rg2_control, handle_rg2_control)
    # spin() keeps Python from exiting until node is shutdown
    print("rg2_gripper controller is ready!")
    rospy.spin()

if __name__ == "__main__":
    rg2_control_server()