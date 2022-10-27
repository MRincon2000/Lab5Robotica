import rospy
import numpy as np
import time
import roboticstoolbox as rtb
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

robot= rtb.DHRobot([
    rtb.RevoluteDH(d=0.045, alpha= -np.pi/2),
    rtb.RevoluteDH(a= 0.105),
    rtb.RevoluteDH(a=0.105),
    rtb.RevoluteDH(a=0.110)
], name= "Pincher")


def cinemInversa(x,y,z):
    l1=0.105
    l2=0.105
    xaux=np.sqrt(x**2+y**2)-0.110
    zaux=-(0.095-z)
    costheta3=(xaux**2+zaux**2-l1**2-l2**2)/(2*l1*l2)
    sentheta3=np.sqrt(1-costheta3**2)
    theta3=np.arctan2(sentheta3, costheta3)
    theta2=np.arctan2(zaux,xaux)-np.arctan2(l2*np,l2*sentheta3, l1+l2*costheta3)
    theta1=np.arctan2(y,x)
    theta4=(theta2-theta3)
    angulos=[theta1, theta2, theta3, theta4, 0]
    return angulos

def enviarPosicion(puntos):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)

    for i in len(puntos):
        x=puntos(i)(0)
        y=puntos(i)(1)
        z=puntos(i)(2)

        state = JointTrajectory()
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        point = JointTrajectoryPoint()
        point.positions = cinemInversa(x,y,z)
        point.time_from_start = rospy.Duration(0.5)
        state.points.append(point)
        pub.publish(state)
        rospy.sleep(1)





def joint_publisher():
    
    posicionActual=[0, 0, -np.pi/2, -np.pi/2, 0]
    while True:
        
        print("Ingrese la tarea que quiere que el robot haga: ")
        print("1. Trazar alcance máximo")
        print("2. Trazar alcance mínimo")
        print("3. Iniciales de los nombres")
        print("4. Triangulo")
        print("5. Circulo")
        print("6. Lineas paralelas")
        print("7. Puntos equidistantes")
        print("8. Figura")

        tarea=input()

        if tarea==1:
            posicionActual=[0, 0, 0, 0, 0]

        elif tarea==2:
            posicionActual=[0, 0, 0, 0, 0]
        elif tarea==3:
            puntos=[[48,-211,0],[48.-211,-3],[23,-195.5,-3],[51,-185.5,-3],[55.5,-201.5,-3],[34,-226,5,-3],[59.5,-217,6,-3],[90,-247,-3],[90,-247,0]]
        elif tarea==4:

        elif tarea==5:

        elif tarea==6:

        elif tarea==7:

        elif tarea==8:

        else : break




if __name__ == '__main__':
    try:
        time.sleep(1)
        joint_publisher()
    except rospy.ROSInterruptException:
        pass