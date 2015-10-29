note
	description: "Calculations of errors/ differences."
	author: "Marius Grimm"
	date: "05.10.15"

class
	ERROR

feature -- Access

	dist_error (x_g: REAL_64; y_g: REAL_64; trig_math: separate TRIGONOMETRY_MATH; od_sig: separate ODOMETRY_SIGNALER): REAL_64
		-- Calculate distance error.
		do
			Result := trig_math.sqrt ((x_g-od_sig.x)^2+(y_g-od_sig.y)^2)
			io.put_string ("%Ndist_error: ")
			io.put_double (Result)
--			io.put_string ("%N")
		end

	theta_error (x_g: REAL_64; y_g: REAL_64; trig_math: separate TRIGONOMETRY_MATH; od_sig: separate ODOMETRY_SIGNALER): REAL_64
		-- Calculate orientation error.
		local
			th_goal: REAL_64
		do
			th_goal := trig_math.atan2 ((y_g-od_sig.y),(x_g-od_sig.x))
			Result := th_goal - od_sig.theta
			io.put_string ("%NOrientation error (theta_error): ")
			io.put_double (Result)
			io.put_string ("%NX: ")
			io.put_double (od_sig.x)
			io.put_string (" Y: ")
			io.put_double (od_sig.y)
--			io.put_string ("%No_sig.theta: ")
--			io.put_double (od_sig.theta)
		end

end -- class ERROR
