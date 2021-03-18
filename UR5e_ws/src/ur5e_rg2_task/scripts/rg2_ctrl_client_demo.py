#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import print_function #_future_ must be put at the beginning

from ur5e_rg2_task.srv import *
import rospy

def rg2_control_client(width, force):
    rospy.wait_for_service('rg2_control')
    
    try:
        # create a handle to the rg2_control service
        rg2_control_srv = rospy.ServiceProxy('rg2_control', rg2_control)
        resp1 = rg2_control_srv(width, force)

        if not resp1.success:
            raise Exception("test failure")
            return 0
        else:
            print("rg2 control success!")
            return resp1.success
    except rospy.ServiceException:
        print("Service call failed")

if __name__ == "__main__":
    argv = rospy.myargv()
    x = float(argv[1])
    y = float(argv[2])
    rg2_control_client(x, y)