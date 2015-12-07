I. Functionality
-------------------------------

This application implements a particle filter localization algorithm. It is able to localize 
a robots position due to a correlation between the map and the sight of a laser scan. The 
particles are displayed as pose array and converge towards the real robots position.

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
* localization_node.cpp						- Main file
* localization_processor.cpp					- Processor class with callbacks
* motion_update.cpp      					- Class for updating the motion of the particle
* sensor_update.cpp						- Class for updating the weights of the particles due to the laser scan
* visualization.cpp 						- Class for visualization (eg marker, posearray, ..)
* distribution/normal_distribution.cpp				- Class with implementation of normal distribution
* distribution/normal_distribution_approximation.cpp		- Class with a approx. of normal distribution
* distribution/triangular_distribution.cpp			- Class with implementation of triangular distribution
* distribution/triangular_distribution_approximation.cpp	- Class with a approx. of triangular distribution
* sampling/stochastic_uniform_sampling.cpp			- Class with not yet implemented stochastic uniform sampling
* sampling/roulette_sampling.cpp				- Class with roulette wheel sampling
* util/math_util.cpp						- Math help class

In the /<your-nethz-name>/object_recognition/parameter/ directory:
* parameter_bag.yaml                        	 - Contains parameter needed on highest level of the hirachy and in callbacks
* motion_update_bag.yaml               		 - Contains parameter specific for the motion update
* sensor_update_bag.yaml               		 - Contains parameter specific for the sensor update
* distribution_bag.yaml               		 - Contains parameter specific for the distributions
* resample_bag.yaml   				 - Contains parameter specific for the resampling
* visualization_bag.yaml               		 - Contains parameter specific for visualization

In the /<your-nethz-name>/localization/launch/ directory:
* localization.launch            		 - Launch file with node configurations and parameter paths

In the /<your-nethz-name>/localization/include/ directory:
* localization_processor.h		- Initializations of .cpp files in /src/ directory
* motion_update.h      			- 
* sensor_update.h			- 
* resample.h				-
* distribution/distribution.h		- 
* visualization.h 			-
* util/math_util.h			-

In the /<your-nethz-name>/localization/include/parameter/ directory:
* parameter_bag.h			- Parameter implementation/ parsing
* motion_update_bag.h			-
* sensor_update_bag.h			-
* distribution_bag.h			-
* resample_bag.h		        -
* recognition/cloud_correlation.h       - 
* visualization_bag.h	                - 

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
