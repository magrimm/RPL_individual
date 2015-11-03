note
	description: "Parameters class for path planning."
	author: "Marius Grimm"
	date: "02.11.15"

expanded class
	PATH_PLANNING_PARAMETERS

feature -- Parameters

	start_x: REAL_64

	start_y: REAL_64

	start_z: REAL_64

	goal_x: REAL_64

	goal_y: REAL_64

	goal_z: REAL_64

	val_inflate: REAL_64

	connected_path_strategy: INTEGER


	set_start_x (a_val: REAL_64)
		do
			start_x := a_val
		end

	set_start_y (a_val: REAL_64)
		do
			start_y := a_val
		end

	set_start_z (a_val: REAL_64)
		do
			start_z := a_val
		end

	set_goal_x (a_val: REAL_64)
		do
			goal_x := a_val
		end

	set_goal_y (a_val: REAL_64)
		do
			goal_y := a_val
		end

	set_goal_z (a_val: REAL_64)
		do
			goal_z := a_val
		end

	set_val_inflate (a_val: REAL_64)
		do
			val_inflate := a_val
		end

	set_connected_path_strategy (a_val: INTEGER)
		do
			if a_val = 4 or a_val = 8 then
				connected_path_strategy := a_val
			else
				io.put_string ("ERROR: connectivity strategy unkown!%N")
			end
		end

end -- class
