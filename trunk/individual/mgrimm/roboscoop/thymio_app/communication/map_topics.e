note
	description: "List of used ROS topics for path planning."
	author: "Marius Grimm"
	date: "29.10.15"

class
	MAP_TOPICS

feature

	-- Used topics.
	map: STRING_8 = "/map"
	path: STRING_8 = "/path"

	s_open: STRING_8 = "/S_open"
	s_closed: STRING_8 = "/S_closed"

end -- class MAP_TOPICS
