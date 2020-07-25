#! /usr/bin/env python
'''
    Help Turtlebot Robot get out of the maze - Final Project ROS in 5 days
    Jonatan Salinas - Last edition: 07/25/20
    
    LaserTurtleSub class
    This class has a subscriber to the /kobuki/laser/scan topic, in order to get
    LaserScan data and store it into a list.
    Also, this class defines the logic that the robot uses to make the next
    movement, according to the obstacles around it.
'''

import rospy
from sensor_msgs.msg import LaserScan

class LaserTurtleSub(object):
    datosDistancias = list()
    
    def __init__(self):
        self.sub_laser = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.my_callback_laser)
        self.distanciaMinima = 1
        self.distanciaMinFrenteLado = 1
        self.distanciaMinimaExtrema = 0.4
        self.valorResta = 0.6

    def devuelveDatosDistancias(self):
        return self.datosDistancias

    #This method returns the direction to which the robot
    #is going to turn in the next movement.
    def haciaDondeGiro(self):
        misDatos = list()
        misDatos = self.devuelveDatosDistancias()

        izqTemp = misDatos[0]
        mediaIzq = misDatos[1]
        frenTemp = misDatos[2]
        mediaDer = misDatos[3]
        derTemp = misDatos[4]

        restaIzq = abs(frenTemp - izqTemp)
        restaDer = abs(frenTemp - derTemp)

        rospy.logdebug("izq: " + str(izqTemp) + " mIzq: " + str(mediaIzq) + " fren: " + str(frenTemp) + " mDer: " + str(mediaDer) + " der: " + str(derTemp))
        #rospy.logdebug("izq: " + str(izqTemp) + " fren: " + str(frenTemp) + " der: " + str(derTemp))

        if mediaIzq == float("inf") or mediaDer == float("inf"):
            if mediaIzq == float("inf") and mediaDer == float("inf"):
                return "libre"
            else:
                if mediaIzq == float("inf"):
                    return "mediaIzquierda"
                else:
                    return "mediaDerecha"

        if  izqTemp < self.distanciaMinima:
            rospy.loginfo("Posible choque a la izquierda")
            if frenTemp < self.distanciaMinFrenteLado:
                rospy.loginfo("Quiza choque al frente tambien")
            if izqTemp < self.distanciaMinimaExtrema:
                rospy.loginfo("Distancia minima extrema detectada a la izquierda")
                return "ejeDerecha"
        if  frenTemp < self.distanciaMinima:
            rospy.loginfo("Posible choque al frente")
        if  derTemp < self.distanciaMinima:
            rospy.loginfo("Posible choque a la derecha")
            if frenTemp < self.distanciaMinFrenteLado:
                rospy.loginfo("Quiza choque al frente tambien")
            if derTemp < self.distanciaMinimaExtrema:
                rospy.loginfo("Distancia minima extrema detectada a la derecha")
                return "ejeIzquierda"    

        if izqTemp > frenTemp and izqTemp > derTemp:
            if restaIzq < self.valorResta:
                if frenTemp < self.distanciaMinFrenteLado:
                    rospy.loginfo("Voy a girar media izquierda")
                    return "mediaIzquierda"
                else:              
                    return "esperaPoco"        
            else:
                rospy.loginfo("Voy a girar a la izquierda")
                return "izquierda"
        elif frenTemp > derTemp and frenTemp > izqTemp:
            rospy.loginfo("Voy pal frente")
            return "frente"
        elif derTemp > izqTemp and derTemp > frenTemp:
            if restaDer < self.valorResta:
                if frenTemp < self.distanciaMinFrenteLado:
                    rospy.loginfo("Voy a girar media derecha")
                    return "mediaDerecha"
                else:
                    return "esperaPoco"
            else:
                rospy.loginfo("Voy a girar a la derecha")
                return "derecha"
        else:
            rospy.loginfo("Parece que tengo el panorama libre")
            return "libre"

    #Each time this callback is executed, some values of the ranges field in the LaserScan message are
    #stored in the datosDistancias list.
    def my_callback_laser(self, msg):
        self.datosDistancias = [msg.ranges[719], msg.ranges[539], msg.ranges[359], msg.ranges[179], msg.ranges[0]]

#main to test the class functionality.
if __name__=="__main__":
    rospy.init_node("sub_nodo_laser", log_level=rospy.DEBUG)
    miObjetoPrueba = LaserTurtleSub()

    unaRate = rospy.Rate(2)

    rospy.timer.sleep(3)

    while not rospy.is_shutdown():
        aDondeGiro = miObjetoPrueba.haciaDondeGiro()
        unaRate.sleep()
            