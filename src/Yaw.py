#!/usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import Float64

class Yaw:
    def __init__(self):

        self.setpoint = 0
        self.plantState = 0
        self.plantTopic = ""

        self.setpointSub = rospy.Subscriber("/yawSetpoint", Float64, self.subCB)
        self.plantSub = rospy.Subscriber("/none", Float64, self.plantCB)
        while(not self.updatePlantTopic()):
            rospy.sleep(0.01)

        self.plantPub = rospy.Publisher('/yawPlantStateNorm', Float64, queue_size=5)
        self.setpointPub = rospy.Publisher('/yawSetpointNorm', Float64, queue_size=5, latch=True)
        self.prevPlantNorm = 0
        rospy.loginfo("init")

    def updatePlantTopic(self):
        topic = rospy.get_param("/TOPICS/YAW")
        if(len(topic) == 0):
            return False
        else:
	    if(topic != self.plantTopic):
	      self.plantSub = rospy.Subscriber(topic, Float64, self.plantCB)
	      self.plantTopic = topic
            return True

    def plantCB(self, data):
        self.plantState = data.data % 360

    def subCB(self, data):
        self.setpoint = data.data % 360

    def getNormalizedPlant(self):
	
        self.updatePlantTopic()
	absDist = abs(self.setpoint-self.plantState) % 360
	d = min(absDist, 360 - absDist)
	expected = (self.setpoint+d) % 360
	if expected  == self.plantState:
		sign = -1	
	else:
		sign = 1
	#rospy.logwarn("set %d, plant %d, d %d, expected %d, out %d", self.setpoint, self.plantState, d, expected, sign*d)
	return d * sign
	

def main():
    rospy.init_node('yaw_normalize')
    yaw = Yaw()
    yaw.setpointPub.publish(data=0)

    while not rospy.is_shutdown():

        yaw.updatePlantTopic()
        plantNorm = yaw.getNormalizedPlant()
        if(plantNorm != yaw.prevPlantNorm):
            yaw.plantPub.publish(data=plantNorm)

        rospy.sleep(.05)
        prevPlantNorm = plantNorm


if __name__ == '__main__':
    main()
