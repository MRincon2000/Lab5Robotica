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
    zaux=-0.063+z
    costheta3=(xaux**2+zaux**2-l1**2-l2**2)/(2*l1*l2)
    print(costheta3)
    sentheta3=np.sqrt(1-costheta3**2)
    theta3=np.arctan2(sentheta3, costheta3)
    theta2=(np.arctan2(zaux,xaux)+np.arctan2(l2*sentheta3, l1+l2*costheta3))
    theta1=np.arctan2(y,x)
    theta4=(-theta2+theta3)
    angulos=[theta1, -(np.pi/2-theta2), -(theta3), (theta4), 0]

    return angulos

def enviarPosicion(puntos,flag):
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
        posActual=cinemInversa(x,y,z)
        if flag==True: posActual[4]=-2
        else: posActual[4]=0
        point.positions = posActual
        point.time_from_start = rospy.Duration(0.5)
        state.points.append(point)
        pub.publish(state)
        rospy.sleep(3)

def enviarAngulos(angulos):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    point = JointTrajectoryPoint()
    point.positions = angulos
    point.time_from_start = rospy.Duration(0.5)
    state.points.append(point)
    pub.publish(state)
    rospy.sleep(3)



def joint_publisher():
    
    posicionHome=[0, 0, -np.pi/2, 0, 0]
    enviarAngulos(posicionHome)
    puntos=[[0.020,-0.260,0.12],[0.020,-0.260,0.028]]
    enviarPosicion(puntos,False)
    puntos=[[0.020,-0.260,0.028],[0.020,-0.260,0.12]]
    enviarPosicion(puntos,True)
    posicionHome=[0, 0, -np.pi/2, 0, -2]
    enviarAngulos(posicionHome)
    
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
            puntos=[[0.047,-0.29,0.08],[0.047,-0.29,0.0],[0.013,0.2945,0.0],[0.013,0.2945,0.08]]
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)

        elif tarea==2:
            puntos=[[-0.071,-0.164,0.06],[-0.071,-0.164,0.0],[0.0075,0.178,0.0],[0.0075,0.178,0.06]]
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)
        elif tarea==3:
            puntos=[[0.048,-0.2645,0.06],[0.048,-0.2645,0.0],[0.023,-0.1955,0.0],[0.051,-0.1855,0.0],[0.064,-0.187,0.0],[0.071,-0.2065,0.0],[0.0595,-0.2175,0.0],[0.0345,-0.2265,0.0],[0.059, -0.2175,0.0],[0.090,-0.247,0.0],[0.090,-0.247,0.06],[0.102,-0.2425,0.06],[0.102,-0.2425,0.0],[0.0775,-0.176,0.0],[0.1015,-0.191,0.0],[0.110,-0.164,0.0],[0.1335,-0.230,0.0],[0.1335,-0.230,0.06]]
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)
        elif tarea==4:
            puntos=[[0.209,-0.1645,0.06],[0.209,-0.1645,0.0],[0.157,-0.091,0.0],[0.246,-0.083,0.0],[0.209,-0.1645,0.0],[0.209,-0.1645,0.06]]
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)
        elif tarea==5:
            angulo=0
            puntos=[]
            puntos.append([0.229+0.04*np.cos(np.deg2rad(24)),-0.0008+0.04*np.sin(np.deg2rad(24)),0.06])

            for o in range(0,17):
                angulo=angulo+24
                puntos.append([0.229+0.04*np.cos(np.deg2rad(angulo)),-0.0008+0.04*np.sin(np.deg2rad(angulo)),0.0])
            puntos.append([0.229+0.04*np.cos(np.deg2rad(o)),-0.0008+0.04*np.sin(np.deg2rad(o)),0.06])
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)
        elif tarea==6:
            puntos=[[0.170,0.072,0.08],[0.170,0.072,0.0],[0.2535,0.091,0.0],[0.2535,0.091,0.08],[0.166,0.090,0.08],[0.166,0.090,0.0],[0.2495,0.108,0.0],[0.2495,0.108,0.08],[0.162,0.107,0.08],[0.162,0.107,0.0],[0.246,0.126,0.0],[0.246,0.126,0.08]]
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)
        elif  tarea==7:
            puntos=[[0.141,0.1285,0.06],[0.141,0.1285,0.0],[0.141,0.1285,0.06],[0.130,0.167,0.06],[0.130,0.167,0.0],[0.130,0.167,0.06],[0.169,0.157,0.06],[0.169,0.157,0.01],[0.169,0.157,0.06],[0.158,0.196,0.06],[0.158,0.196,0.01],[0.158,0.196,0.06],[0.1965,0.186,0.06],[0.1965,0.186,0.01],[0.1965,0.186,0.06]]
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)
        elif tarea==8:
            angulo=0
            puntos=[]
            puntos.append([0.068+0.04*np.cos(np.deg2rad(24)),0.227+0.04*np.sin(np.deg2rad(24)),0.10])

            for o in range(0,27):

                angulo=angulo+360/26
                radio=0.04+0.01*np.sin(4*np.deg2rad(angulo))
                puntos.append([0.068+radio*np.cos(np.deg2rad(angulo)),0.227+radio*np.sin(np.deg2rad(angulo)),0.0])
            puntos.append([0.068+radio*np.cos(np.deg2rad(o)),0.227+radio*np.sin(np.deg2rad(o)),0.10])
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)
        else : 
            enviarAngulos(posicionHome)


            puntos=[[0.02,-0.25,0.12],[0.02,-0.25,0.025]]
            enviarPosicion(puntos,True)
            puntos=[[0.02,-0.25,0.025],[0.02,-0.25,0.12]]
            enviarPosicion(puntos,False)
            posicionHome=[0, 0, -np.pi/2, 0, -2]
            enviarAngulos(posicionHome)

            posicionHome=[0, 0, -np.pi/2, 0, 0]
            enviarAngulos(posicionHome)
            break




if __name__ == '__main__':
    try:
        time.sleep(1)
        joint_publisher()
    except rospy.ROSInterruptException:
        pass