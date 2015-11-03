
note
	description: "Application in Roboscoop."
	author: "Marius Grimm"
	date: "28.10.15"

class
	APP

inherit
	ROS_ENVIRONMENT

create
	make

feature {NONE} -- Initialization

	make
			-- Create and launch the robot.
		local
			robo_node: separate ROBOSCOOP_NODE
			ros_spinner: separate ROS_SPINNER
			path_planner: PATH_PLANNING

		do
			-- Initialize this application as a ROS node.
			robo_node := (create {ROS_NODE_STARTER}).roboscoop_node
			synchronize (robo_node)

			-- Listen to ROS.
			create ros_spinner.make
			start_spinning (ros_spinner)

			-- Parse parameter text file
			create parser
			params_path := Arguments.argument (1).to_string_8
			params := parser.read_parameters (create {STRING}.make_from_separate (params_path))

			-- Create a path_planner object.
			create path_planner.make (params)

			-- Launch path planning in path_planner.		
			path_planner.plan_path
		end

feature {NONE}

	parser: PATH_PLANNING_PARSER
			-- Parser class for paramteters from text file.

	params: PATH_PLANNING_PARAMETERS
			-- All Parameters needed.

	params_path: separate STRING

end -- class APP
