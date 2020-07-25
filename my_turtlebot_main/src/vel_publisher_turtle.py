#! /usr/bin/env python
'''
    Help Turtlebot Robot get out of the maze - Final Project ROS in 5 days
    Jonatan Salinas - Last edition: 07/25/20
    
    MoveTurtlePub class
    This class has a publisher to the /cmd_vel topic, in order to make
    the robot move in a determined time.
    In this class, the methods for the robot to move to a specific
    direction are defined.
'''

import rospy
from geometry_msgs.msg import Twist

class MoveTurtlePub(object):

    def __init__(self):
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.myRateVel = rospy.Rate(2)
        self.myTurtleTwist = Twist()        #Message to publish Twist data into the /cmd_vel topic
        self.velocidadRobot = 0.3           #General velocity of the robot
        self.velocidadXGiro = 0.1           #Robot velocity in x axis when it is turning
        self.duracionVuelta = 4.5           #Seconds the robot takes to make a turn
        self.duracionMediaVuelta = 2.25     #Seconds the robot takes to make a half-turn

    #Method to publish the Twist data into the /cmd_vel topic.
    #It ensures that the value is published.
    def publicarVelocidad(self):
        bandera = False

        while not bandera:
            connections = self.pub_vel.get_num_connections()
            if connections > 0:
                self.pub_vel.publish(self.myTurtleTwist)
                bandera = True
            else:
                self.myRateVel.sleep()

    #Method to turn left with velocity also in x axis
    def girarIzquierda(self):
        rospy.loginfo("Girando hacia izquierda...")
        self.myTurtleTwist.linear.x = self.velocidadXGiro
        self.myTurtleTwist.angular.z = self.velocidadRobot
        self.publicarVelocidad()
        rospy.timer.sleep(self.duracionVuelta)
        self.myTurtleTwist.angular.z = 0
        self.myTurtleTwist.linear.x = self.velocidadRobot
        self.publicarVelocidad()

    #Method to turn right with velocity also in x axis
    def girarDerecha(self):
        rospy.loginfo("Girando hacia derecha...")
        self.myTurtleTwist.linear.x = self.velocidadXGiro
        self.myTurtleTwist.angular.z = self.velocidadRobot*-1
        self.publicarVelocidad()
        rospy.timer.sleep(self.duracionVuelta)
        self.myTurtleTwist.angular.z = 0
        self.myTurtleTwist.linear.x = self.velocidadRobot
        self.publicarVelocidad()

    #Method to turn right (half) with less velocity in x axis
    def medioGiroDerecho(self):
        rospy.loginfo("Girando hacia media derecha...")
        self.myTurtleTwist.linear.x = self.velocidadXGiro
        self.myTurtleTwist.angular.z = self.velocidadRobot*-1
        self.publicarVelocidad()
        rospy.timer.sleep(self.duracionMediaVuelta)
        self.myTurtleTwist.angular.z = 0
        self.myTurtleTwist.linear.x = self.velocidadRobot
        self.publicarVelocidad()        

    #Method to turn left (half) with less velocity also in x axis
    def medioGiroIzquierdo(self):
        rospy.loginfo("Girando hacia media izquierda...")
        self.myTurtleTwist.linear.x = self.velocidadXGiro
        self.myTurtleTwist.angular.z = self.velocidadRobot
        self.publicarVelocidad()
        rospy.timer.sleep(self.duracionMediaVuelta)
        self.myTurtleTwist.angular.z = 0
        self.myTurtleTwist.linear.x = self.velocidadRobot
        self.publicarVelocidad()

    #Method to turn right (half) without velocity in x axis
    def giroEjeDerecha(self):
        rospy.loginfo("Girando hacia eje derecha...")
        self.myTurtleTwist.linear.x = 0
        self.myTurtleTwist.angular.z = self.velocidadRobot*-1
        self.publicarVelocidad()
        rospy.timer.sleep(self.duracionMediaVuelta)
        self.myTurtleTwist.angular.z = 0
        self.myTurtleTwist.linear.x = self.velocidadRobot
        self.publicarVelocidad()

    #Method to turn left (half) without velocity in x axis
    def giroEjeIzquierda(self):
        rospy.loginfo("Girando en eje izquierda...")
        self.myTurtleTwist.linear.x = 0
        self.myTurtleTwist.angular.z = self.velocidadRobot
        self.publicarVelocidad()
        rospy.timer.sleep(self.duracionMediaVuelta)
        self.myTurtleTwist.angular.z = 0
        self.myTurtleTwist.linear.x = self.velocidadRobot
        self.publicarVelocidad()        

    #Method to make the robot move forward
    def avanza(self):
        self.myTurtleTwist.linear.x = self.velocidadRobot
        self.publicarVelocidad()

    #Method to make the robot move forward and wait 1.3sec
    def avanzaPoco(self):
        self.myTurtleTwist.linear.x = self.velocidadRobot
        self.publicarVelocidad()
        rospy.timer.sleep(1.3)

    #Method to assign 0 to velocity in x axis
    def frena(self):
        self.myTurtleTwist.linear.x = 0
        self.publicarVelocidad()

    #Method to test some of the methods above
    def probarMoveTurtlePub(self):
        rospy.loginfo("Probando clase MoveTurtlePub...")
        self.myTurtleTwist.linear.x = self.velocidadRobot
        rospy.loginfo("Adelante...")
        self.publicarVelocidad()
        rospy.timer.sleep(3)
        self.myTurtleTwist.linear.x = 0
        self.publicarVelocidad()
        rospy.timer.sleep(1)
        self.myTurtleTwist.linear.x = self.velocidadRobot*-1
        rospy.loginfo("Atras...")
        self.publicarVelocidad()
        rospy.timer.sleep(3)
        self.myTurtleTwist.linear.x = 0
        self.publicarVelocidad()
        rospy.timer.sleep(1)
        rospy.loginfo("Prueba de MoveTurtlePub completada.")

#main, to test the class functionality
if __name__=="__main__":
    rospy.init_node("MoveTurtlePub_nodo_prueba")
    rospy.loginfo("Prueba de la clase MoveTurtlePub.")
    unPublisherTurtle = MoveTurtlePub()

    #unPublisherTurtle.probarMoveTurtlePub()
    #unPublisherTurtle.girarIzquierda()
    #unPublisherTurtle.avanza()

    rospy.loginfo("Prueba acabada.")


    