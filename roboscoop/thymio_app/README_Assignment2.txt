I. Functionality
-------------------------------

This application implements the path planning from a starting coordinate to a goal
coordinate. It first creates from a given grid a graph. Aftwards, a search algorithm 
(A* in the implemented version) to find the optimal path. The path is then published 
to ROS and can be displayed in Rviz.

II. Usage
-------------------------------

First start $roscore in your terminal. Second, you have to start the map_server where 
your map is going to be read from. For this type in another terminal: 

	$ rosrun map_server map_server [~/PATH]

Afterwards start Rviz in another terminal with the command:

	$ rosrun rviz rviz

Finally, you can open Eiffel and execute the application. To change the staring position,
goal position, the obstacle inflation or the connectivity strategy, please change accordingly 
in the path_planning_parameters.txt file.
 

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


In the /<your-nethz-name>/roboscoop/thymio_app directory:
* app.e 		- main file of the project 

In the /<your-nethz-name>/roboscoop/thymio_app/rpl directory:
* path_planning.e       	- Path planner abstraction class
* graph_builder.e       	- Class for creating a graph out of a grid
* search_algorithm.e    	- Class for possible path optimisation algorithms to use
* four_connected_path.e   	- Connectivity strategy with four connected path
* eight_connected_path.e  	- Connectivity strategy with eight connected path
* path_planning_parser.e  	- Parser class for path planning
* path_planning_parameters.e 	- Parameter class for path planning
* path_planning_parameters.txt	- Paramters for path planning
* path_publisher.e		- Class for publishing a PATH_MSG
* map_topics.e			- Used ros topics
* cost.e			- Heuristic cost class


IV. How it works?
-------------------------------

APPLICATION class creates an instance of path planner where the implemented path planning 
is called. This behaviour calls the graph_builder to make the graph from the grid.
Afterwards the search algorithm is called which finds the optimal path. The class 
PUBLISHER is responsible for finally publishing the PATH_MSG.

