#! /usr/bin/env python
'''
    Help Turtlebot Robot get out of the maze - Final Project ROS in 5 days
    Jonatan Salinas - Last edition: 07/25/20
    
    OdomTurtleSub class
    This class has a subscriber to the /odom topic, in order to get
    Odometry data. This data is stored into a class variable.
    The Odometry Action Server uses an instance of this object
    to get the odometry data.
'''

import rospy
from nav_msgs.msg import Odometry

class OdomTurtleSub(object):
    miDatoOdometria = Odometry()

    def __init__(self):
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.my_callback_odom)
        
    #Method that returns the odometry data stored.
    def devuelveDatoOdometria(self):
        return self.miDatoOdometria

    #Each time the callback is executed, the odometry message read is stored into a variable.
    def my_callback_odom(self, msg):
        self.miDatoOdometria = msg

#main to verify the class functionality.
if __name__=="__main__":
    rospy.init_node("sub_nodo_odom", log_level=rospy.INFO)
    miObjetoPrueba = OdomTurtleSub()

    unaRate = rospy.Rate(1)
    odomPrueba = Odometry()

    while not rospy.is_shutdown():
        odomPrueba = miObjetoPrueba.devuelveDatoOdometria()
        print(odomPrueba)
        unaRate.sleep()