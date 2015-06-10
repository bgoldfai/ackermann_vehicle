#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

joy = Joy()
receivedJoy = False

def callback(data):    
  global joy
  joy = data
  global receivedJoy
  receivedJoy = True
  
def talker():

  throttleMultiplier = 0.5
  steeringMultiplier = 20
  throttleAxis = 5
  brakeAxis = 2
  steeringAxis = 3

  msg = AckermannDriveStamped()
  msg.drive.speed = 0.0
  msg.drive.acceleration = 0.0
  msg.drive.steering_angle = 0.00
  msg.drive.steering_angle_velocity = 0.0
  
  ackPub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=1)
  rospy.Subscriber('joy', Joy, callback)
  rospy.init_node('joystickAckermann')
  
  rate = rospy.Rate(10) # 10hz
  
  while not rospy.is_shutdown():
    if receivedJoy:
      msg.header.stamp = rospy.Time.now()
      
      #R2 to increase speed
      if joy.axes[throttleAxis]+1 < 1.95:
        msg.drive.speed += throttleMultiplier*(-joy.axes[throttleAxis]+1)
      #L2 for to decrease speed
      if joy.axes[brakeAxis]+1 < 1.95:
        msg.drive.speed -= throttleMultiplier*(-joy.axes[brakeAxis]+1)
      
      #L1 or R1 to set speed to 0
      if joy.buttons[4] or joy.buttons[5]:
        msg.drive.speed = 0.0
      
      msg.drive.steering_angle = math.radians(steeringMultiplier)*joy.axes[steeringAxis]
      
      
      if joy.buttons[0] == 1:
        msg.drive.speed -= 0.5
      elif joy.buttons[3] == 1:
        msg.drive.speed += 0.5;
      elif joy.buttons[1] == 1 or joy.buttons[2] == 1:
        msg.drive.speed = 0.0;
      
      ackPub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
