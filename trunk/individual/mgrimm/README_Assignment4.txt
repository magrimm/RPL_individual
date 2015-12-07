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

In the /<your-nethz-name>/localization/src/ directory:
* localization_node.cpp						- 
* localization_processor.cpp					- 
* motion_update.cpp      					- 
* sensor_update.cpp						- 
* visualization.cpp 						- Class for visualization (eg marker, posearray, ..)
* distribution/normal_distribution.cpp				-
* distribution/normal_distribution_approximation.cpp		-
* distribution/triangular_distribution.cpp			-
* distribution/triangular_distribution_approximation.cpp	-
* sampling/stochastic_uniform_sampling.cpp			-
* sampling/roulette_sampling.cpp				-
* util/math_util.cpp						-

In the /<your-nethz-name>/object_recognition/parameter/ directory:
* parameter_bag.yaml                        	 - Contains parameter needed on highest level of the hirachy and in callbacks
* motion_update_bag.yaml               		 - Contains parameter specific for the motion update
* sensor_update_bag.yaml               		 - Contains parameter specific for the sensor update
* distribution_bag.yaml               		 - Contains parameter specific for the distributions
* resample_bag.yaml   				 - Contains parameter specific for the resampling
* visualization_bag.yaml               		 - Contains parameter specific for visualization

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
The parameter structure allows the user to parse around the specific sets of parameters.
With the help of rosparam the parameters can be easily saved in a .yaml file.
The parameter implementation/parsing is done with the help of the header file structure in 
/<your-nethz-name>/object_recognition/include/parameter/ directory. The parameter_bag.h file 
represents the hightest level of parameter and contains structs with lower level of parameter
as for example filter_bag. Within this eg filter_bag is again struct subset which contain
the parameter of the specific eg filter. This parameter can be accessed in filter/<<>>.h .

2)
The code itself is structured in a clear pattern. The main() initialises ros and the parameters.
Furthermore, the subscription to the camera topic is done in which the callback function is 
called over and over again.
The callback function is implemented in the class cloud_handling. A database of spin images of the 
.pcd files for the objects is created. In the callback itself all filters (class cloud_filter)
are going to be appliedto the original pointcloud received by the camera. After resolution, 
passthrough and outlier filtering the segmentation takes place. The remaining poincloud has to 
be divided into different clusters if more than one object is to be seen. For this a euclidean 
cluster extraction (class cloud_segmentation) is applied. Each cluster of the scene is going
to be compared now with the database of the objects. For that spin images are also being created
for the scene. After all we receive a correspondance value between (-1,1) which represents the 
quality of fit. The objects can be compared now directly to the scene and the highest value
above a certain threshold should represent now in theory the object. 
Finally, a visualization maker (class cloud_visualization) is displayed in a certain colour to
indicate the recognised object.


V. Limitations
-------------------------------

A good finetuning of the tuning knobs is require to get a reasonable result. Also certain 
thresholds for eg an unknown object are limiting the precision of the process. 
Furthermore, is it important to have good .pcd files of the object to build a reliable 
database of the objects.
