# Tritech-Micron-Driver
Driver and Sonar parser for the Tritech Micron Driver 

#Introḍuction

The software for the Micron/SeaSprite sonar is located in the package micron_driver_ros. This package includes two nodes: 


* Micron_driver_ros_node: This is the actual sonar driver and handles the setup and polling of the device 

* Scanline Parser:  This node subscribes to the information provided by the sonar driver and processes it.

#Nodes: 

##Micron_driver_ros: 

##Configuration:
The initial configuration of the node is made through a .yaml file located in the /config folder. The following parameters can be set: 
* Frame ID: Name of the frame of reference in which the scanline topic will be published
* Port_: Serial port to which the Sonar is connected
* Num_bins_: Number of points along a bin on which a sound intensity measurement will be taken. 
* Range_: Maximum range  
* Velocity of sound: Sound speed in the water
* Angle_step_size: Defines the angle that the sonar head advances between two consecutive steps. It is given as 1/16 of a gradian. A gradian is 0.9º.
* leftLimit: Left limit of the scanned sector 
* rightLimit: Right limit of the scanned sector
* use_debug_mode: If activated,  the debug mode prints a lot of additional information to the screen. Not recommended for normal use.
* simulate: Allows to simulate the sonar. Not Working right now 

##Subscriptions 
The driver node  isn't subscribed to any topic. 

##Publications:
The node publishes messages of the type Scanline every time the sonar reports a scanned bin on the topic /micron_driver/scanline

##Services
The driver has a service that allows to reconfigure the sonar without the need to restart the whole node.

#Scanline parser:

##Description:
This node subscribes to the Scanline topic published by the Sonar driver. Each time a bin is published, the Scanline parser processed that bin and publishes a message in the ROS standard pointCloud and LaserScan types  
 
##Configuration:
The conversion to a PointCloud message type can be tuned with the following parameters:  
* min_distance_threshold:  The scanline parser will ignore any echoes that have a distance to the sonar smaller than this value. It allows us to discard false echoes that happen close to the sonar.
* min_point_cloud_intensity_threshold: Its the minimum echo intensity that has to be found on the bin to be considered a valid point and published. 
* use_point_cloud_threshold: If set to false the scanline parser will not look for a minimum intensity and publish all echoes as points as long as they are over the minimum distance threshold.
* only_first_point: If set to true, only the first point that is over both the distance and intensity threshold will be taken as valid and published. All other points on the bin will be ignored. With it, we can obtain an approximation to the line of constant echo intensity defined by min_point_cloud_intensity_threshold. 

The conversion to a LaserScan message type can be tuned with the following parameters: 

The LaserScan conversion isn't fully implemented yet!!  

##Subscriptions:  
The scanline_parser node is only subscribed to the topic /micron_driver/scanline

##Publications:
The node publishes ScanLine and PointCloud messages. 

##Services
It has a service that allows to reconfigure the scanline parser without the need to restart the whole node.



#Launch Files: 

In the  micron_driver_ros/launch folder there are two different launch files:

* MicronDriver.launch: This file plays a recorded bag with sonar scanlines and initiates a Scanline Parser node. It also launches RVIZ to visualize the data.
* MicronDriver_Complete.launch: This file initiates both the sonar driver and the scanline parser.
 
