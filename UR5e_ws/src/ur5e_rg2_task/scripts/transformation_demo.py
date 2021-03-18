from geometry_msgs.msg import Quaternion
from tf_conversions.transformations import quaternion_from_euler
import math

q1 = quaternion_from_euler(-2.735, 1.561, -2.744)
q_rot = quaternion_from_euler(-math.pi/2, 0, 0)
q2 = quaternion_multiply(q_rot, q1)
q2.normalize()


