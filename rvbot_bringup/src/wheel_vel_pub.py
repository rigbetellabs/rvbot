#!/usr/bin/env python
# license removed for brevity
import rospy
import message_filters
from std_msgs.msg import Float32
from rvbot_bringup.msg import MotorSpeed


pub = rospy.Publisher('motor_speeds', MotorSpeed, queue_size=10)

def callback(leftspeed, rightspeed):
    speeds = MotorSpeed()
    speeds.lspeed = leftspeed.data
    speeds.rspeed = rightspeed.data
    pub.publish(speeds)

def listener():
    rospy.init_node('wheel_vel_pub', anonymous=True)

    left_sub = message_filters.Subscriber('/left_wheel_speed', Float32)
    right_sub = message_filters.Subscriber('/right_wheel_speed', Float32)

    ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass