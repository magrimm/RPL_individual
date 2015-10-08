note
	description: "Class for level of robot"
	author: "Marius Grimm"
	date: "05.10.15"

class
	ROBOT

create
	make

feature {NONE} -- Initialization

	make
			-- Create a robot.
		do
			-- Initialize sensors.
			create odometry_sig.make_with_topic ({THYMIO_TOPICS}.odometry)

			-- Initialize actuators.
			create diff_drive.make_with_topic ({THYMIO_TOPICS}.velocity)

			-- Initialize behaviours.
			create going_to_goal_behaviour.make_with_attributes (odometry_sig, diff_drive)
		end

feature -- Access

	start_going_to_goal
		-- Start moving towards goal position.
		do
			start_behaviour (going_to_goal_behaviour)
		end

	stop_going_to_goal
		-- Stop moving.
		do
			stop_behaviour (going_to_goal_behaviour)
		end

feature {NONE} -- Robot parts

	odometry_sig: separate ODOMETRY_SIGNALER
			-- Current state of the odometry.

	diff_drive: separate THYMIO_DIFFERENTIAL_DRIVE
			-- Differential drive.

feature {NONE} -- Behaviours

	going_to_goal_behaviour: separate PLANNER
		-- Behaviour for moving to goal.

	start_behaviour (a: separate PLANNER)
		-- Launch 'a'.
		do
			a.start
			io.put_string ("Start moving towards goal.%N")
		end

	stop_behaviour (a: separate PLANNER)
		-- Stop 'a'.
		do
			a.stop
	--		synchronize (a)
			io.put_string ("Stop moving.%N")
		end

end -- class ROBOT
