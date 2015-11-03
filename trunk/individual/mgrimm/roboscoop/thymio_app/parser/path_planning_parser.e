note
	description: "Parser class for path planning."
	author: "Marius Grimm"
	date: "02.11.15"

class
	PATH_PLANNING_PARSER

feature

	read_parameters (file_path: STRING): PATH_PLANNING_PARAMETERS
		local
			input_file: PLAIN_TEXT_FILE
			string_tokens: LIST[STRING]
			params: PATH_PLANNING_PARAMETERS
	  	do
	  		create params
	    	create input_file.make_open_read (file_path)

	    	from
	    		input_file.read_line
	    	until
	    		input_file.exhausted
	    	loop
	    		string_tokens := input_file.last_string.split (':')

    		if (string_tokens.at (1).is_equal ("start_x")) then
	    			params.set_start_x (string_tokens.at (2).to_real_64)
	    		elseif (string_tokens.at (1).is_equal ("start_y")) then
	    			params.set_start_y (string_tokens.at (2).to_real_64)
	    		elseif (string_tokens.at (1).is_equal ("start_z")) then
	    			params.set_start_z (string_tokens.at (2).to_real_64)
	    		elseif (string_tokens.at (1).is_equal ("goal_x")) then
	    			params.set_goal_x (string_tokens.at (2).to_real_64)
	    		elseif (string_tokens.at (1).is_equal ("goal_y")) then
	    			params.set_goal_y (string_tokens.at (2).to_real_64)
	    		elseif (string_tokens.at (1).is_equal ("goal_z")) then
	    			params.set_goal_z (string_tokens.at (2).to_real_64)
	 	    	elseif (string_tokens.at (1).is_equal ("val_inflate")) then
	    			params.set_val_inflate (string_tokens.at (2).to_real_64)
	    		elseif (string_tokens.at (1).is_equal ("connected_path_strategy")) then
	    			params.set_connected_path_strategy (string_tokens.at (2).to_integer_32)
	    		end

	    		input_file.read_line
	    	end
	    	Result := params
	  	end

end -- class
