


<h1 align="center"> Universidad Nacional de Colombia <br>
Facultad de Ingeniería <br>
Depto. de Ing. Mecánica y Mecatrónica <br>
Robótica <br>
Laboratorio 5 - Robótica- Cinemática Inversa con robot Phantom x Pincher <br>
Miguel Angel Rincón <br>
Robinson Ordúz </h1>

## introducciòn:

La cinemática directa usa ecuaciones cinemáticas para determinar la posición del actuador 
final a partir de la longitud de los eslabones y el ángulo que forman en sus articulaciones.
En una cadena cinemática serial como el robot Pincher, un conjunto de vectores de posición siempre 
tiene única solución, es decir un solo punto en el espacio.

## variables de los eslabones:

![pincher](https://user-images.githubusercontent.com/20913010/196850773-730bd781-cd5c-484d-932f-6ba7e6c11571.PNG)
d1= 45 mm;
d2= 105 mm;
d3= 105 mm;
d4= 40 mm;

## Tabla DH:

![dh pincher](https://user-images.githubusercontent.com/20913010/196851289-b248f673-c7ce-4791-9a60-b52783aa7214.PNG)


<h1 align="center"> Programas desarrollados en python</h1>
<h2>Requisitos previos </h2>

Para que los programas hechos en Python sean funcionales se requiere hacer varios pasos previos, los cuales serán listados a continuación:
- Instalar Catkin Build: Catkin build es un programa complementario a ROS, el cual permite la administración de paquetes o librerías que facilitan bastante el uso de ROS y su integración con los motores Dynamixel, presentes en el Pincher Phantom X.
- Instalar el paquete px_robot: Este paquete puede ser administrado mediante Catkin Build. Este paquete realiza la comunicación con los 5 motores Dynamixel AX-12 mediante ROS, crea los nodos necesarios y permite acceder a los topicos y servicios ofrecidos por estos nodos. El paquete se encuentra disponible en el enlace:  https://github.com/felipeg17/px_robot.


<h2>Cinemática inversa </h2>

Para calcular la cinemática inversa del manipulador de manera que este sea capaz de alcanzar los puntos deseados, se sabe que los eslabones 2 y 3 conforman un submecanismo de doble péndulo, por tanto es posible obtener los ángulos 2 y 3 de forma sencilla. Para el ángulo 1, se sabe que este es el único que gira alrededor del eje Z, y por tanto su ecuación se obtiene con una sencilla tangente inversa. Se tiene para el ángulo 1:

![theta1](https://user-images.githubusercontent.com/49238418/200097116-cfa0d460-0ec4-458e-874b-0a76186ed8d5.png)

Por tanto, su ecuación es la siguiente:

```python
  
  theta1=np.arctan2(y,x)
    
```
Para obtener las ecuaciones de los angulos 2 y 3 se tiene lo siguiente:

![pendulo doble](https://user-images.githubusercontent.com/49238418/200097188-3305c484-638c-456f-a4e6-38c28768f909.png)

- Se calcula una distancia horizontal total Xaux aplicando teorema de la norma, como se busca que la cuarta articulación siempre se mantenga en posición horizontal, entonces a esta distancia se le resta la longitud del eslabón 4. El resultado es la distancia horizontal que debe alcanzar el péndulo doble:

```python
  xaux=np.sqrt(x**2+y**2)-0.110 
```
- La distancia vertical Zaux que debe alcanzar el doble péndulo tiene en cuenta la altura del primer eslabón, la base y el marcador, esto provoca que haya un desface de altura negativo de 6.3 cm:

```python
  zaux=-0.063+z
```
- A continuación se obtienen las ecuaciones para el angulo 3 en configuración de codo arriba:

```python
    costheta3=(xaux**2+zaux**2-l1**2-l2**2)/(2*l1*l2)
    sentheta3=np.sqrt(1-costheta3**2)
    theta3=np.arctan2(sentheta3, costheta3)
```
- Y a partir de estas, se obtiene la ecuación para el angulo 2:


```python
   theta2=(np.arctan2(zaux,xaux)+np.arctan2(l2*sentheta3, l1+l2*costheta3))
```

- Se debe tener en cuenta que este ángulo es con respecto a la horizontal, por lo tanto al enviarlo al robot se debe sumar pi/2 rad debido a que la primera articulación es vertical.

- Para calcular el ángulo 4, simplemente se sabe que esta debe mantener el eslabón en posición horizontal, debido a ello la ecuación es sencillamente la siguiente:

```python
  theta4=(-theta2+theta3)
```
Se crea un arreglo con estos angulos y se retornan. La herramienta se envia abierta por defecto. Las articulaciones 2, 3 y 4 están invertidas en el robot, por tal razón es necesario retornarlas negativas. El método completo es el siguiente:


```python
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
```
<h2>Publicador</h2>

Se utiliza el método de publicador utilizado en el laboratorio pasado, esto debido a que permite el movimiento simultáneo de las articulaciones, al cual se le hacen dos variantes, el primero recibe el punto deseado para calcular la cinemática inversa y transmitir la solución y el segundo el cual recibe los ángulos directamente y los transmite. Ademas se crea una variable flag, la cual indica si la herramienta se encuentra abierta a cerrada:

- Por cinemática inversa: 

```python
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
```
- Por cinemática directa:
```python

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

```

<h2>Obtención de puntos </h2>

Para obtener los puntos que representan las trayectorias deseadas se hizo inicialmente el dibujo sobre la base que sostiene el robot de forma manual, se le tomó una foto, a la cual se le hicieron mediciones para mantener la relación con el tamaño real:
![IMG_20221021_121030](https://user-images.githubusercontent.com/49238418/200097732-31396b06-914b-4c5e-9870-94932e7690a6.jpg)

A continuación esta imagen se importó a AutoCAD y de esta manera se hicieron los dibujos que se busca que el robot realice:

![Autocad](https://user-images.githubusercontent.com/49238418/200097793-3ac3af35-8bc3-41d4-a528-a64919939443.png)

De esta manera, con la ayuda de AutoCAD se obtuvieron los puntos deseados para cada figura, los cuales se pueden apreciar en el archivo de hoja de cálculo incluido. Estos puntos se introducen como listas de triplas, las cuales contienen la coordenada x,y,z. Esta coordenada z se introduce teniendo en cuenta donde se quiere que el manipulador dibuje y donde no.

Se tienen dos casos especiales. Para el círculo y la figura libre se utilizó un método mas automatizado. Se tiene el centro obtenido de AutoCAD y el radio. Con esto se define un número de puntos o divisiones de estas figuras y se dividen los 360° en este número de puntos, a continuación, cada uno de estos se genera sumando el radio multiplicado por el seno y coseno del correspondiente ángulo a las coordenadas del centro. El resultado para el círculo del cual se quieren 16 puntos es:

```python

            angulo=0
            puntos=[]
            puntos.append([0.229+0.04*np.cos(np.deg2rad(24)),-0.0008+0.04*np.sin(np.deg2rad(24)),0.06])

            for o in range(0,17):
                angulo=angulo+24
                puntos.append([0.229+0.04*np.cos(np.deg2rad(angulo)),-0.0008+0.04*np.sin(np.deg2rad(angulo)),0.0])
            puntos.append([0.229+0.04*np.cos(np.deg2rad(o)),-0.0008+0.04*np.sin(np.deg2rad(o)),0.06])
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)

```

Para el caso de la figura libre finalmente se optó por generar un tŕebol de cuatro hojas convirtiendo el radio en una función sinusoidal de amplitud de 1cm. El resto del procedimiento es el mismo únicamente teniendo en cuenta que para este caso se dibujaron 26 puntos:

```python
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
```

Por último se establece una posición de Home, la cual es la siguiente:


```python
  posicionHome=[0, 0, -np.pi/2, 0, -2]
```
![home2](https://user-images.githubusercontent.com/49238418/200098100-c3513558-d374-49fe-90d5-abe527ae9767.png)

También se crea una secuencia de pasos para que el robot sea capaz de tomar y soltar el marcador, para ello se definen una serie de puntos y se juega con la variable flag para abrir o cerrar la herramienta. 

![marcador](https://user-images.githubusercontent.com/49238418/200098175-b2ea8fe7-be98-4c27-8adf-6c4dee4d0ba2.png)

Se escribe un ciclo while infinito el cual permite seleccionar que figura dibujar. La rutina finalmente es la siguiente:

```python
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

```
El funcionamiento del robot con las rutinas deseadas se puede ver en el siguiente enlace: 

[![Video](https://img.youtube.com/vi/UE6lGZyHkr0/0.jpg)](https://www.youtube.com/watch?v=UE6lGZyHkr0&ab_channel=MiguelRinc%C3%B3n)


Y el resultado obtenido:
![IMG_20221103_160351](https://user-images.githubusercontent.com/49238418/200098582-dd452bc3-9587-49f7-80fc-cd3b1f5b31e8.jpg)

<h2>Calculo de error </h2>

Para calcular el error de escritura se tomarán como base los dibujos del triángulo y el círculo, mediante la ayuda de programa Adobe Photoshop, se calculan las áreas de cada una de estas dos figuras conociendo su área deseada obtenida analíticamente y contrastándola con el área obtenida en la realidad.

Para esto, Photoshop incorpora la herramienta Análisis mediante la cual se pueden obtener medidas reales de una imagen. Se procede a determinar la escala de la imagen con su tamaño real. Para ello se toma el diámetro del circulo interior y sabiendo que el del dibujado en AutoCAD es de 357 mm entonces se obtiene una relación entre pixeles y mm, la cual en este caso es: 1924 pixeles=357 mm.

![image](https://user-images.githubusercontent.com/49238418/200101071-40b3be5e-25c3-4e65-b8cd-793246b97982.png)

Una vez hecho esto se selecciona el círculo mediante la herramienta de selección rápida y se da clic en la opción de grabar medida. Se repite el procedimiento para el caso del triángulo. Las áreas obtenidas son 5004.3801 mm2 para el círculo y 2877.6916 mm2 para el triángulo.

![image](https://user-images.githubusercontent.com/49238418/200101278-e61e929b-84c1-4922-afad-d7a2d87cd542.png)

Analíticamente, se sabe que el círculo deseado debía tener un área de 5026.5 mm2 debido a que su radio es de 40 mm. Calculando el error porcentual para el área en este caso se obtiene (5004.3801-5026.5)/5026.5*100%=-0.44%. Lo cual es un error bastante reducido. 

Por otro lado, la situación para el triángulo es muy diferente. Como se tiene un lado de 90mm, el triángulo analíticamente debe poseer un área de 3507.402 mm mientras que en el dibujo tiene un área de 2877.6916. En este caso repitiendo el procedimiento anterior se obtiene (2877.6916-3507.402)/3507.402*100%=-17.95%. En perímetro se esperará una longitud de 270 mm mientras que en el dibujo es de 277.046. Repitiendo el procedimiento (277.046-270)/270*100%=2.609%

<h2>Conclusiones </h2>

- Se puede intuir que la gran diferencia en los errores calculados se debe a que en el caso del círculo, se dibujaron 16 puntos (lados) diferentes mientras que para el triángulo unicamente se transmitieron los extremos de cada uno de los lados, de esta manera la trayectoria evidentemente fue considerablemente menos exacta provocando un error mucho mas elevado. 
- Para obtener errores mas bajos lo recomendable es interpolar las trayectorias con muchos mas puntos de los que se tomaron. A su vez, debido a que el rbot utilizado no permite control de velocidad, este es posible de "simular" variando la distribucion y distancia entre cada uno de los puntos.
- La cinemática inversa permite lograr trayectorias complejas en el robot Pincher mediante ecuaciones sencillas siguiendo el método geométrico.
- El uso de herramientas gráficas como AutoCAD y Photoshop resulta de gran ayuda para el diseño y validación de las trayectorias.
