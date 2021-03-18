#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rg2_gripper
import sys

# if __name__ == "_main_" :
#夹住积木
width = sys.argv[1]
force = sys.argv[2]
rg2_gripper.setGripperWidth(width, force)