#! /usr/bin/env python
'''
    Help Turtlebot Robot get out of the maze - Final Project ROS in 5 days
    Jonatan Salinas - Last edition: 07/25/20
    
    Main program
    Here, a service client for the Obstacle service server and a client
    for the Odometry Action Server are created.
    According to the direction to turn returned by the obstacle service
    server, the robot will execute a specific method to move. These until
    the action is still 'alive'.

'''
import rospy
import actionlib
from vel_publisher_turtle import MoveTurtlePub
from my_turtlebot_actions.msg import record_odomAction, record_odomGoal
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

DONE = 2

if __name__=="__main__":
    rospy.init_node("main_turtlebot_node", log_level=rospy.DEBUG)
    unaRate = rospy.Rate(2)

    #Creation of the action client
    miCliente_AccionOdom = actionlib.SimpleActionClient('/myActionOdomServer', record_odomAction)
    rospy.loginfo("Esperando al Action Server Odom...")
    miCliente_AccionOdom.wait_for_server()
    rospy.loginfo("Action Server Odom encontrado!")

    #creation of the service client connection
    rospy.wait_for_service('/obstacle_service_server')
    rospy.loginfo("Obstacle service server hallado")
    conexionCliente_ObsServ = rospy.ServiceProxy('/obstacle_service_server', Trigger)

    cosaMoveTurtle = MoveTurtlePub()
    
    rospy.loginfo("Hasta aca todo bien!!")
    
    miCliente_AccionOdom.send_goal(record_odomGoal())   #A goal is sended to the odometry action server

    #The program will continue until the action is cancelled, the time runs out or the turtlebot
    #has exited the maze
    while miCliente_AccionOdom.get_state() < DONE:
        resServidorObs = conexionCliente_ObsServ(TriggerRequest())
        aDondeGiro = resServidorObs.message     #direction for the robot to turn

        if aDondeGiro == "izquierda":           #According to the direction, the robot will execute a
            cosaMoveTurtle.girarIzquierda()     #specific method
        elif aDondeGiro == "mediaIzquierda":
            cosaMoveTurtle.medioGiroIzquierdo()
        elif aDondeGiro == "frente":
            cosaMoveTurtle.avanza()
        elif aDondeGiro == "derecha":
            cosaMoveTurtle.girarDerecha()
        elif aDondeGiro == "mediaDerecha":
            cosaMoveTurtle.medioGiroDerecho()
        elif aDondeGiro == "ejeIzquierda":
            cosaMoveTurtle.giroEjeIzquierda()
            cosaMoveTurtle.avanzaPoco()
        elif aDondeGiro == "ejeDerecha":
            cosaMoveTurtle.giroEjeDerecha()
            cosaMoveTurtle.avanzaPoco()
        elif aDondeGiro == "esperaPoco":
            rospy.loginfo("Que me espere dice")
        else:
            rospy.loginfo("Me sigo porque creo que tengo todo libre")
    
        unaRate.sleep()

    cosaMoveTurtle.frena()
    rospy.loginfo("El programa ha terminado.")
        
    #Code to print the Odometry data stored during the robot's journey along the maze:
    #It could be discommented if it is useful to see the data.
    
    '''
    resultadoAccion = miCliente_AccionOdom.get_result()

    for i in range(0, len(resultadoAccion.result_odom_array)):
        rospy.loginfo("Odom data " + str(i) + ":")
        print(resultadoAccion.result_odom_array[i])

    rospy.loginfo("El resultado de la accion odom quedo como: " + str(miCliente_AccionOdom.get_state()))
    '''