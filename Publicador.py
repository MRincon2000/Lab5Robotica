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
    print(costheta3)
    sentheta3=np.sqrt(1-costheta3**2)
    theta3=np.arctan2(sentheta3, costheta3)
    theta2=(np.arctan2(zaux,xaux)-np.arctan2(l2*sentheta3, l1+l2*costheta3))
    theta1=np.arctan2(y,x)
    theta4=(-theta2-theta3)
    angulos=[theta1, theta2, theta3, theta4, -1]
    return angulos

def enviarPosicion(puntos):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)

    for i in range(len(puntos)):
        puntoActual=puntos[i]
        x=puntoActual[0]
        y=puntoActual[1]
        z=puntoActual[2]

        state = JointTrajectory()
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        point = JointTrajectoryPoint()
        point.positions = cinemInversa(x,y,z)
        point.time_from_start = rospy.Duration(0.5)
        state.points.append(point)
        pub.publish(state)
        rospy.sleep(2)





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

        tarea=int(input())

        if tarea==1:
            posicionActual=[0, 0, 0, 0, 0]

        elif tarea==2:
            posicionActual=[0, 0, 0, 0, 0]
        elif tarea==3:
            puntos=[[0.048,-0.2645,0.0],[0.048,-0.2645,-0.03],[0.023,-0.1955,-0.03],[0.051,-0.1855,-0.03],[0.064,-0.187,-0.03],[0.071,-0.2065,-0.03],[0.0595,-0.2175,-0.03],[0.0345,-0.2265,-0.03],[0.059, -0.2175,-0.03],[0.090,-0.247,-0.03],[0.090,-0.247,0.0]]
            enviarPosicion(puntos)
        else : break




if __name__ == '__main__':
    try:
        time.sleep(1)
        joint_publisher()
    except rospy.ROSInterruptException:
        pass