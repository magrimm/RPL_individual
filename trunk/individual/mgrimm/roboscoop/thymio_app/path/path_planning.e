note
	description: "Planner."
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
			create occupancy_grid_sig.make_with_topic ({MAP_TOPICS}.map)
			-- Publish path
			create path_planner.make_with_topic ({MAP_TOPICS}.path)
		end

feature -- Access

	plan_path
		local
			grid_to_graph: GRAPH_BUILDER
		do
			create grid_to_graph

			-- Publish path
			path_planner.set_path_with_spatial_graph_nodes (grid_to_graph.grid_to_graph (occupancy_grid_sig), "map")
		end

feature {NONE} -- Implementation

	occupancy_grid_sig: separate OCCUPANCY_GRID_SIGNALER
	path_planner: PATH_PUBLISHER

end -- class
