note
	description: "Lowest level of control pipeline: PID"
	author: "Marius Grimm"
	date: "08.10.15"

class
	CONTROLLER

inherit
	CANCELLABLE_CONTROL_LOOP

create
	make

feature {NONE} -- Initialization

	make (s_sig: separate STOP_SIGNALER; x_g: REAL_64; y_g: REAL_64)
			-- Create Current and assign given attributes.
		do
			stop_signaler := s_sig
			create trigonometry_math

			-- For calculation of loop times for I and D part of controller
			create timer.make (10000)
			create curr_theta_time
			create prev_theta_time
			create curr_dist_time
			create prev_dist_time
			-- For calculation of errors
			create curr_theta_error
			create prev_theta_error
			create curr_dist_error
			create prev_dist_error
		end

feature {NONE} -- Implementation

	trigonometry_math: separate TRIGONOMETRY_MATH
			-- Math libary with arctan.	

	timer: separate TIMER
			-- Time class for calculating looptime.

feature -- Constants

	prev_theta_time, curr_theta_time: REAL_64
	prev_dist_time, curr_dist_time: REAL_64
	curr_theta_error, prev_theta_error: REAL_64
	curr_dist_error, prev_dist_error: REAL_64

	-- goal tolerance
	tol: REAL_64 = 0.005

	-- Controller parameters
	Kp_theta: REAL_64 = 0.5 -- 0.6
	Ki_theta: REAL_64 = 0.6 -- 0.5
	Kd_theta: REAL_64 = 0.6 -- 0.03
	Kp_dist: REAL_64 = 0.7 	-- 0.8
	Ki_dist: REAL_64 = 0.2 -- 0.05
	Kd_dist: REAL_64 = 0.1 	-- 0.1

	-- Speed basis
	vtheta: REAL_64 = 0.0
	vx: REAL_64 = 0.0

feature {PLANNER}

	go (err: separate ERROR; s_sig: separate STOP_SIGNALER; drive: separate THYMIO_DIFFERENTIAL_DRIVE; odom_sig: separate ODOMETRY_SIGNALER;
			feature_cont: separate FEATURE_CONTROLLER; x_g: REAL_64; y_g: REAL_64)
		do
				-- Controlling angular velocity
			if  err.dist_error (x_g, y_g, trigonometry_math, odom_sig) > tol then
				drive.set_velocity (pid_dist (err, stop_signaler, odom_sig, timer, x_g, y_g),
										pid_theta (err, stop_signaler, odom_sig, feature_cont, timer, x_g, y_g))
				-- Run features
				feature_cont.features_go_to_goal

			-- Stop when in tolerance range of goal coordinates	
			else
				drive.stop
				s_sig.set_stop_requested (TRUE)

				-- Change features to "goal_reached".
				feature_cont.features_goal_reached
			end
		end

feature {NONE} -- Controller

	pid_theta (err: separate ERROR; s_sig: separate STOP_SIGNALER;
				odom_sig: separate ODOMETRY_SIGNALER; f_cont: separate FEATURE_CONTROLLER;
				t: separate TIMER; x_g: REAL_64; y_g: REAL_64): REAL_64
		-- PID controller for angular velocity (theta_error).
		local
			looptime: REAL_64
			P_theta, I_theta, D_theta: REAL_64

		do
			curr_theta_time := t.current_time_millis
			looptime := t.current_time_millis - prev_theta_time
			io.put_string ("%Nlooptime_theta: ")
			io.put_double (looptime)
			curr_theta_error := err.theta_error (x_g, y_g, trigonometry_math, odom_sig)
			io.put_string ("%Nprev_theta_error: ")
			io.put_double (prev_theta_error)

			-- PID
			P_theta := Kp_theta * curr_theta_error
			I_theta := I_theta + Ki_theta * (looptime * curr_theta_error)
			if looptime /= 0 then
				-- looptime/1000 to get seconds
				D_theta := Kd_theta * (curr_theta_error - prev_theta_error) / (looptime/1000)
			else
				D_theta := 0
			end

			-- Output P I D
			io.put_string ("%NP_theta: ")
			io.put_double (P_theta)
			io.put_string ("   D_theta: ")
			io.put_double (D_theta)
			io.put_string ("   I_theta: ")
			io.put_double (I_theta)
			io.put_string ("%N")

			-- Total angular velocity
			Result := vtheta + (P_theta + I_theta + D_theta)

			-- Updating error and time
			prev_theta_error := curr_theta_error
			prev_theta_time := curr_theta_time
		end

	pid_dist (err: separate ERROR; s_sig: separate STOP_SIGNALER;
				odom_sig: separate ODOMETRY_SIGNALER; t: separate TIMER; x_g: REAL_64; y_g: REAL_64): REAL_64
		-- PID controller for linear velocity (distance error --> 2D euklidean distance).
		local
			looptime: REAL_64
			P_dist, I_dist, D_dist: REAL_64

		do
			curr_dist_time := t.current_time_millis
			looptime := curr_dist_time - prev_dist_time
			io.put_string ("%Nlooptime_dist: ")
			io.put_double (looptime)
			curr_dist_error := err.dist_error (x_g, y_g, trigonometry_math, odom_sig)

			-- PID
			P_dist := Kp_dist * curr_dist_error
			I_dist := I_dist + Ki_dist * (looptime * curr_dist_error)
			if looptime /= 0 then
				-- looptime/1000 to get seconds
				D_dist := Kd_dist * (curr_dist_error - prev_dist_error) / (looptime/1000)
			else
				D_dist := 0
			end

			-- Output P I D
			io.put_string ("%NP_dist: ")
			io.put_double (P_dist)
			io.put_string ("   D_dist: ")
			io.put_double (D_dist)
			io.put_string ("   I_dist: ")
			io.put_double (I_dist)
			io.put_string ("%N")

			-- Total linear velocity
			Result := vx + (P_dist + I_dist + D_dist)

			-- Updating error and time
			prev_dist_error := curr_dist_error
			prev_dist_time := curr_dist_time
		end

end -- class CONTROLLER_V5

