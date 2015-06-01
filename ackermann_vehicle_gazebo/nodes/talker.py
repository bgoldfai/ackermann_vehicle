#!/usr/bin/env python
# license removed for brevity
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

def talker():
    pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(3) # 10hz
    while not rospy.is_shutdown():
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.speed = 3.5
        msg.drive.steering_angle = -0.02
        
#        msg.drive.acceleration = 0.2
#        msg.drive.steering_angle_velocity = 0.2

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
