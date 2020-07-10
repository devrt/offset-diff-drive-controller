#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("test_omni_square", anonymous=True)

pub = rospy.Publisher("/robot/offset_diff_drive_controller/cmd_vel", Twist, queue_size=1)

while pub.get_num_connections() == 0:
    rospy.sleep(1)

cmd_vel = Twist()

cmd_vel.linear.x = 0.05
cmd_vel.linear.y = 0.0
rospy.loginfo('publish command')
pub.publish(cmd_vel)
rospy.sleep(10)

cmd_vel.linear.x = 0.0
cmd_vel.linear.y = 0.05
rospy.loginfo('publish command')
pub.publish(cmd_vel)
rospy.sleep(10)

cmd_vel.linear.x = -0.05
cmd_vel.linear.y = 0.0
rospy.loginfo('publish command')
pub.publish(cmd_vel)
rospy.sleep(10)

cmd_vel.linear.x = 0.0
cmd_vel.linear.y = -0.05
rospy.loginfo('publish command')
pub.publish(cmd_vel)
rospy.sleep(10)

cmd_vel.linear.x = 0.0
cmd_vel.linear.y = 0.0
rospy.loginfo('publish stop command')
pub.publish(cmd_vel)
