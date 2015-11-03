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

end -- class MAP_TOPICS
