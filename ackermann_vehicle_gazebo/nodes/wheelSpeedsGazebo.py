#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from auto_rally_msgs.msg import wheelSpeeds
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState


class WheelSpeedsGazebo(object):

  def getWheelSpeed(self, data, name, dia):
    try:
      idx = data.name.index(name)
      #return data.twist[idx].angular.y * (dia)/2.0 #if data is from linkState message
      return data.velocity[idx]*(dia/2.0)
    except IndexError:
      rospy.logerror('modelStates does not contain ' + name)
      return 0.0
    
  
  def callback(self, data):    
    ws = wheelSpeeds()
    ws.header.stamp = rospy.Time.now()
    ws.header.frame_id = 'gazeboSim'
    
    #print data
    ws.lfSpeed = self.getWheelSpeed(data, self.left_front_name, self.left_front_dia)
    ws.rfSpeed = self.getWheelSpeed(data, self.right_front_name, self.right_front_dia)
    ws.lbSpeed = self.getWheelSpeed(data, self.left_rear_name, self.left_rear_dia)
    ws.rbSpeed = self.getWheelSpeed(data, self.right_rear_name, self.right_rear_dia)
    
    if self.pub:
      self.pub.publish(ws)
      

  def getJointStateWheelParams(self, lr_prefix, fr_prefix):
    dia = rospy.get_param('/ackermann_vehicle/ackermann_controller/' + lr_prefix + '_' + fr_prefix + '_wheel/diameter')
    name = lr_prefix + '_' + fr_prefix + '_axle'
    return name, dia
    
  def getLinkStateFrontWheelParams(self, lr_prefix):
    dia = rospy.get_param('/ackermann_vehicle/ackermann_controller/' + lr_prefix + '_front_wheel/diameter')
    name = 'ackermann_vehicle::' + rospy.get_param('/ackermann_vehicle/ackermann_controller/' + lr_prefix + '_front_wheel/steering_link_name')
    return name, dia

  def getLinkStateRearWheelParams(self, lr_prefix):
    dia = rospy.get_param('/ackermann_vehicle/ackermann_controller/' + lr_prefix + '_rear_wheel/diameter')
    name = 'ackermann_vehicle::' + rospy.get_param('/ackermann_vehicle/ackermann_controller/' + lr_prefix + '_rear_wheel/link_name')
    return name, dia

  def __init__(self):
   
    rospy.init_node('wheelSpeedsGazebo')
    
    self.left_front_name, self.left_front_dia = self.getJointStateWheelParams('left', 'front')
    self.right_front_name, self.right_front_dia = self.getJointStateWheelParams('right', 'front')
    self.left_rear_name, self.left_rear_dia = self.getJointStateWheelParams('left', 'rear')
    self.right_rear_name, self.right_rear_dia = self.getJointStateWheelParams('right', 'rear')
    
    #don't set up callback until params are initialized
    self.pub = rospy.Publisher('/wheelSpeeds', wheelSpeeds, queue_size=1)
    #self.sub = rospy.Subscriber('/ackermann_vehicle/gazebo/link_states', ModelStates, self.callback)
    self.sub = rospy.Subscriber('/ackermann_vehicle/joint_states', JointState, self.callback)
    
if __name__ == "__main__":
  speed = WheelSpeedsGazebo()
  rospy.spin()
