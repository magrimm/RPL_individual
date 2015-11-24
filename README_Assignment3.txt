I. Functionality
-------------------------------

This application implements object recognition with correlation images. It is able to detect, 
filter and distinguish between different objects. Furthermore the application indicates
the detected object by publishing different marker.


II. Usage
-------------------------------

First connect to your camera (in our case a primesense camera) via USB and run the command

	$ roslaunch openni_launch openni.launch

This command also starts ROS, if it is not running yet. Afterwards open the application in 
your preferred IDE and launch the command

	$ roslaunch object_recognition object_recognition.launch 

"Object recognition" represents the package_name and "object_recognition.launch" the launch
file. The .lauch file specifies the nodes which are going to be started and the parameter 
paths are defined there. 
 

III. File/directory structure
-------------------------------

Class Hierachy

   |---------|           |----------|	       |----------|
   |   APP   |---------->|   ROBOT  |--------->|  PLANNER |
   |---------|           |----------|	       |----------|	      
						     |
						     |
						     | 
						     \/
                                             |--------------|
                                             |  CONTROLLER  |
                                             |--------------|
                     			        ^        ^ 
					       /          \                             
                  			      /            \                            
                                             /              \
                  			    /                \  										
      			    |----------------------|       |------------|
          	            |  FEATURE_CONTROLLER  |       |   ERROR    |
             	            |----------------------|       |------------|


In the /<your-nethz-name>/object_recognition/src/ directory:
* object_recognition_node.cpp		- Main file of the project with executable int main() 
* cloud_handling.cpp			- Processes the callback 
* cloud_filter.cpp      		- Class with filters applied on pointcloud
* cloud_segmentation.cpp		- Class with segmentation techniques applied on pointcloud
* cloud_matching.cpp			- Class for matching/ comparing (with correlatino images) the objects
* cloud_visualization.cpp 		- Class for visualization (eg marker, ...)

In the /<your-nethz-name>/object_recognition/src/parameter/ directory:
* parameter.yaml                        - Contains parameter needed on highest level of the hirachy
* filter/resolution.yaml                - Contains parameter specific for resolution filter
* filter/passthrough.yaml               - Contains parameter specific for passthrough filter
* filter/outlier.yaml                   - Contains parameter specific for statistical outlier removal filter
* segmentation/euclidean_cluster.yaml   - Contains parameter specific for euclidean cluster removal (segmentation)
* recognition/cloud_correlation.yaml    - Contains parameter specific for correlation image object recognition
* visualization/marker.yaml             - Contains parameter specific for visualization of the marker

In the /<your-nethz-name>/object_recognition/launch/ directory:
* object_recognition.launch             - Launch file with node configurations and parameter paths

In the /<your-nethz-name>/object_recognition/include/ directory:
* object_recognition_node.h		- Initializations of .cpp files in /src/ directory
* cloud_handling.h			- 
* cloud_filter.h      			- 
* cloud_segmentation.h			- 
* cloud_matching.h			- 
* cloud_visualization.h 		-

In the /<your-nethz-name>/object_recognition/include/parameter/ directory:
* parameter_bag.h			- Parameter implementation/ parsing
* filter_bag.h				-
* segmentation_bag.h			-
* recognition_bag.h			-
* visualization_bag.h		        -
* filter/resolution_filter.h            - 
* filter/passthrough_filter.h           -  
* filter/outlier_filter.h               - 
* segmentation/euclidean_cluster.h      - 
* recognition/cloud_correlation.h       - 
* visualization/marker.h                - 

IV. How it works?
-------------------------------

1) Parameter structure:


APPLICATION class creates an instance of ROBOT class where the implemented behaviours of
the robot are called. Those behaviours are defined in the class PLANNER. Further, it accesses 
class CONTROLLER where the PID controller for the linear and angular velocity are
implemented. For the calculation of the distance and orientation error the additional 
class ERROR is passing on the data. The light and sound features within the CONTROLLER are
called from the class FEATURE_CONTROLLER.


V. Limitations
-------------------------------

Due to the inaccuracies of the robot THYMIO 2 it is adviced to properly calibrate the robot.
Otherwise the odometry data is misleading and your goal coordinates will not be reached
precisely.
