I. Functionality
-------------------------------

This application implements a particle filter localization algorithm. It is able to localize 
a robots position due to a correlation between the map and the sight of a laser scan. The 
particles are displayed as pose array and converge towards the real robots position.

II. Usage
-------------------------------

Since we use simulated date from a rosbag we first need to play the bag. For that move to
the folder with the bag and execute

	$ rosbag play localization.bag

Furthermore, we need to execute the .launch file of the project. Within this launchfile 
the different nodes to be started are specified and also the paths to the parameter 
.yaml files. For that move to your workspace and execute

	$ roslaunch localization localization.launch

Now the application should be started and rviz should open.
"localization" represents the package_name and "localization.launch" the launch
file.
 

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
/<your-nethz-name>/localization/include/parameter/ directory. The parameter_bag.h file 
represents the hightest level of parameter and may contain structs with lower level of parameter. 

2)
The code itself is structured in a clear pattern. The main() initialises ros and the parameters.
Furthermore, the subscription to the map, odometry and laser scan topic is done in which the 
callback functions are called over and over again.
The callback functions are implemented in the class localization_processor. 
The map callback receives the map message and saves it as an object for later use of the data.
Particles are created distributed over the map with different orientations.
The second callback, the odometry, updates the control vector of the odometry. The odometry of
the last period t-1 and the current period t are saved.
Handling and communicating through signalers the callback odometry and the callback scan are 
always executed sequentially. In the callback scan first the motion update is calculated due to 
the difference in odometry from and noise. Afterwards the sensor update calculates for each 
particle a correlation value. This value represents how good the sight of the particle and 
the global position in the map are correlated. 
According to this weights the particles are going to be resampled in the next step. The 
resampling biases a convergence of the particles towards the particles with high weights.
At the end the particles are visualized in RVIZ using geometry_msgs::PoseArray.


V. Limitations
-------------------------------

A good finetuning of the tuning knobs is required to get a reasonable result. For the sensor
update only the parameters alpha 1-4 need to be chosen properly which are also robot-specific.
Those parameters highly influence the convergence behaviour of the particles.
