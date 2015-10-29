note
	description: "Summary description for {PATH_PLANNING}."
	author: "Marius Grimm"
	date: "29.10.15"

class
	PATH_PLANNING

create
	make

feature {NONE} -- Initialization

	make
		do
			-- Read map
			create map_reader.make_with_topic ({MAP_TOPICS}.map)
			-- Publish path
			create path_planner.make_with_topic ({MAP_TOPICS}.path)
		end

feature

	map_reader: OCCUPANCY_GRID_SIGNALER
	path_planner: PATH_PUBLISHER

feature

	test
		do
			path_planner.set_path (0, 0, 0, 2, 2, 2)
		end



end -- class PATH_PLANNING
