#! /usr/bin/env python
'''
    Help Turtlebot Robot get out of the maze - Final Project ROS in 5 days
    Jonatan Salinas - Last edition: 07/25/20
    
    OdomActionServer class
    This class defines the attributes and methods for an action
    server, with the goal of being saving odometry data and check
    if the robot has exited the maze. When the robot has exited
    the maze or a specific amount of time has passed, the action
    finishes.

    For this Action Server, a custom message (from the file record_odom.action)
    is used. This message only has a field in the result section
    (an array of Odometry data)
'''

import rospy
import actionlib
from my_turtlebot_actions.msg import record_odomAction, record_odomResult
from nav_msgs.msg import Odometry
from odom_subscriber_turtle import OdomTurtleSub
import math

class OdomActionServer(object):

    def __init__(self):
        self.myActionOdomServerObj = actionlib.SimpleActionServer('/myActionOdomServer', record_odomAction, self.actionOdomCallback, False)
        self.myActionOdomServerObj.start()
        self.myRate = rospy.Rate(1)
        self.tiempoMaxSeg = 90              #Time limit for the robot to get out of the maze
        self.miSubOdom = OdomTurtleSub()    #OdomTurtleSub object, used to subscribe to the /odom topic 
        self.miResultadoAct = record_odomResult()
        rospy.loginfo("Odometry Action Server ready!")

    '''
    This method has a while, which counts up to the time limit for exiting the maze.
    If the objective is canceled or the robot exit the maze, the program gets out of the loop.
    Each second (according to myRate variable), an odometry data is stored in the array.
    '''
    def actionOdomCallback(self, goal):
        arrayTempOdom = []
        odomTemp = Odometry()
        success = False

        i = 0
        while i < self.tiempoMaxSeg:
            if self.myActionOdomServerObj.is_preempt_requested():
                self.myActionOdomServerObj.set_preempted()
                rospy.loginfo("El objetivo ha sido cancelado")
                i = self.tiempoMaxSeg
                success = False

            odomTemp = self.miSubOdom.devuelveDatoOdometria()
            arrayTempOdom.append(odomTemp)

            ladoX = odomTemp.pose.pose.position.x
            ladoY = odomTemp.pose.pose.position.y
            hipotenusa = math.sqrt((ladoX*ladoX)+(ladoY*ladoY))

            if hipotenusa > 9.4:
                rospy.loginfo("Parece ser que ya salimos")
                i = self.tiempoMaxSeg
                success = True

            i = i+1
            self.myRate.sleep()
            rospy.logdebug("Han pasado: " + str(i) + " segs")

        self.miResultadoAct.result_odom_array = arrayTempOdom

        if success:
            rospy.loginfo("La accion ha terminado. El robot ha salido del laberinto en tiempo.")
            self.myActionOdomServerObj.set_succeeded(self.miResultadoAct)
        else:
            rospy.loginfo("El objetivo se ha cancelado o se ha terminado el tiempo.")
            self.myActionOdomServerObj.set_succeeded(self.miResultadoAct)

#The node is created and an instance of the object is created
if __name__=="__main__":
    rospy.init_node("action_server_odom_node", log_level=rospy.INFO)
    rospy.loginfo("Iniciando Action Odometry Server.")
    OdomActionServer()
    rospy.spin()