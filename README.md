# Tritech-Micron-Driver
Driver and Sonar parser for the Tritech Micron Driver 

#Introducción:

El software para el sonar Micron/Seasprite se encuentra en el paquete micrón_driver_ros. Este paquete contiene dos nodos: 

* Micrón_driver_ros_node: Este nodo es el propio driver y se encarga de la comunicación con el Sonar. 
* Scanline Parser: Es el encargado de tomar los datos publicados por el driver y convertirlos a un formato útil para el AUV. 



#Nodos: 

##Micron_driver_ros: 

Archivos: 
	

Funcionamiento: 

##Configuración: 
La configuración inicial del nodo se realiza mediante un fichero .yaml ubicado en la carpeta /micron_driver_ros/config.  Los parámetros son los siguientes: 
* Frame ID: Nombre del sistema de cordenadas en el que publicará el topic scanline
* Port_: Puerto al que se encuentra conectado el Sonar 
* Num_bins_: Número puntos en los que se tomará una medida en casa escaneo
* Range_: Alcance máximo del Sonar 
* Velocity of sound: velocidad del sonido en el agua
* Angle_step_size: Define el ángulo que avanza el haz con cada paso. Se da en dieciseisavos de gradianes. ¡Ojo! Un gradian son 0.9º.
* leftLimit: Límite izquierdo del barrido del sonar. 
* rightLimit: Límite derecho del barrido del sonar
* use_debug_mode: En este modo se presenta gran cantidad de información por pantalla y es útil para analizar el correcto funcionamiento del driver. En caso de funcionamiento normal es conveniente tenerlo desactivado
* simulate: Sirve para simular el sonar sin tenerlo conectado. En estos momentos aun no esta operativo este modo de funcionamiento, por lo que hay que dejarlo en false. 

##Suscripciones: 
El nodo del driver no esta suscrito a ningún topic

##Publicaciones:
El nodo publica mensajes de tipo Scanline cada vez que recibe datos de un escaneo del sonar en el tipic /micron_driver/scanline

##Servicios: 
El driver tiene un servicio que permite realizer la reconfiguración del sonar sin tener que reiniciar el driver. 

#Scanline parser:

##Archivos: 

##Funcionamiento:  
El nodo se suscribe al topic en el que el driver del sonar publica los datos de cada barrido. Al producirse una publicación en este topic, el nodo toma esos datos y los convierte al formato estándar de ROS PointCloud y LaserScan. 

##Configuración:
El proceso de conversión a nube de puntos permite ajustar los siguientes parámetros: 
* Distancia mínima de puntos: Este parámetro hace que se ignore la información de intensidad en todos los puntos con una distancia al sonar menor a la indicada. Sirve sobre todo para eliminar ecos indeseados que se pueden producir cerca de la superficie.
* Intensidad mínima de ecos: Es la intensidad de eco mínima necesaria para que el punto se agregue al mensaje de PointCloud y se publique. 
* Uso de intensidad mínima: Permite desactivar el límite mínimo de intensidad de eco. En este caso, todos los puntos, independientemente del eco recibido se añadirán a la nube de puntos una vez superada la distancia mínima
* Solo primer punto: Con este modo puede activarse que solo se publique en la nube de puntos el primer punto encontrado que cumpla las condiciones establecidas por las tres variables anteriores. De esta forma, combinando distancia e intensidad mínima de eco se puede obtener un perfil aproximado de la línea de intensidad de eco constante igual al valor umbral determinado. Puede valer para perfilar el fondo, por ejemplo. 

Para el proceso de conversión a LaserScan se pueden variar los siguientes parámetros: 

##Suscripciones:  
El nodo scanline_parser esta suscrito únicamente al topic /micron_driver/scanline, en el que publica el driver del sonar cada sucesivo escaneo del sonar. 

##Publicaciones:
El nodo publica mensajes del tipo PointCloud como laserscan. E

##Servicios: 
El nodo tiene un servicio de reconfiguración que permite ajustar los parámetros mencionados en el apartado de configuración. 


#Launch Files: 

En la carpeta micron_driver_ros/launch se encuentran tres archivos launch diferentes:
* Record.launch: Este archivo carga el fichero de configuración del sonar, lanza el nodo del sonar y almacena los datos en un archivo .bag para su posterior análisis. 
* Play.launch: Este archivo carga el fichero de configuración del scanline_parser, lanza el nodo y reproduce un archivo .bag grabado mediante el .lauch previamente descrito. Además ejecuta RVIZ con una configuración específica para visualizer los datos procesados. 
* PlayandRecord.launch Este fichero es el que habrá que incorporar al AUV. Se cargan los archivos de configuración tanto del driver como del scanline_parser, y lanzan ambos nodos.
 
