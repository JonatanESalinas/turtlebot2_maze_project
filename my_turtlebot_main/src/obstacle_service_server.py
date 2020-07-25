#! /usr/bin/env python
'''
    Help Turtlebot Robot get out of the maze - Final Project ROS in 5 days
    Jonatan Salinas - Last edition: 07/25/20
    
    ObstacleServiceServer class
    This class defines the attributes and methods of a service server.
    The goal with this server is to determine the direction to which
    the robot is going to turn in the next movement, with the help
    of a LaserTurtleSub object.

'''
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from laser_subscriber_turtle import LaserTurtleSub

class ObstacleServiceServer(object):

    def __init__(self):
        self.my_serviceObstacle = rospy.Service('/obstacle_service_server', Trigger, self.my_callback_obstacle)
        self.myLaserSub = LaserTurtleSub()
        self.myResponse = TriggerResponse()
        rospy.loginfo("Service server de deteccion de obstaculo listo.")

    #When the service is called, this method is executed, which returns the TriggerResponse with
    #the direction to turn, using the LaserTurtleSub object.
    def my_callback_obstacle(self, request):
        aDondeGiro = self.myLaserSub.haciaDondeGiro()
        self.myResponse.success = True
        self.myResponse.message = aDondeGiro
        return self.myResponse

#The node for the service and an instance of the class are
#created.
if __name__=="__main__":
    rospy.init_node('obstacle_service_server_node', log_level=rospy.DEBUG)
    rospy.loginfo("Iniciando Obstacle Service Server...")
    ObstacleServiceServer()
    rospy.spin()