note
	description: "Behaviours"
	author: "Marius Grimm"
	date: "05.10.15"

class
	PLANNER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (odom_sig: separate ODOMETRY_SIGNALER; d_drive: separate THYMIO_DIFFERENTIAL_DRIVE)
		do
			diff_drive := d_drive
			odometry_sig := odom_sig
			
			create stop_sig.make
			create features_controller.make
			create error

			-- Input via command window of goal coordinates
			create x_goal
			create y_goal
			io.put_string ("%Nx_goal: ")
			io.read_double
			x_goal:= io.last_double
			io.put_string ("%Ny_goal: ")
			io.read_double
			y_goal:= io.last_double
		end

feature -- Constants

	-- Goal coordinates
	x_goal: REAL_64
	y_goal: REAL_64

feature -- Access

	start
			-- Start the behaviour.
		local
			a: separate CONTROLLER
		do
			create a.make (stop_sig, x_goal, y_goal)

			sep_stop (stop_sig, False)
			sep_start (a)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_sig, True)
		end

feature {NONE} -- Implementation

	stop_sig: separate STOP_SIGNALER
			-- Signaler for stopping the behaviour.

	diff_drive: separate THYMIO_DIFFERENTIAL_DRIVE
			-- Object to control robot's speed.

	odometry_sig: separate ODOMETRY_SIGNALER
			-- Signaler for Odometry data.

	features_controller: separate FEATURE_CONTROLLER
			-- Class to conrol features like light and sound.

	error: separate ERROR
			-- Error calculation (distance and orientation).

feature {NONE} -- Behaviours

	sep_start (a: separate CONTROLLER)
			-- Start controllers asynchronously.
		do
			a.repeat_until_stop_requested (
				agent a.go (error, stop_sig, diff_drive, odometry_sig, features_controller, x_goal, y_goal))
		end

	sep_stop (s_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for a stop.
		do
			s_sig.set_stop_requested (val)
		end

end -- class PLANNER
