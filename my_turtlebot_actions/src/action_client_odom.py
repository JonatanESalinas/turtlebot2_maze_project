#! /usr/bin/env python
import rospy
import actionlib
from my_turtlebot_actions.msg import record_odomAction, record_odomGoal

DONE = 2

rospy.init_node("action_client_node")
miCliente = actionlib.SimpleActionClient('/myActionOdomServer', record_odomAction)
rospy.loginfo("Esperando al Action Server Odom...")
miCliente.wait_for_server()
rospy.loginfo("Action Server Odom encontrado!")

miCliente.send_goal(record_odomGoal())

unaRate = rospy.Rate(1)

while miCliente.get_state() < DONE:
    rospy.loginfo("Estado de la accion: " + str(miCliente.get_state()))
    unaRate.sleep()

print("Estos son los resultados:")

unResultado = miCliente.get_result()
for i in range(0, len(unResultado.result_odom_array)):
    rospy.loginfo("Odom data " + str(i) + ":")
    print(unResultado.result_odom_array[i])

rospy.loginfo("El estado final quedo como: " + str(miCliente.get_state()))