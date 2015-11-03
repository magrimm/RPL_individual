note
	description: "Planner."
	author: "Marius Grimm"
	date: "29.10.15"

class
	PATH_PLANNING

create
	make

feature {NONE} -- Initialization

	make (params: PATH_PLANNING_PARAMETERS)
		do
			-- Read map
			create occupancy_grid_sig.make_with_topic ({MAP_TOPICS}.map)
			-- Publish path
			create path_pub.make_with_topic ({MAP_TOPICS}.path)

			create graph.make_with_attributes (params)
		end

feature -- Access

	plan_path
		do
			-- Publish path
			path_pub.set_path_with_spatial_graph_nodes (graph.grid_to_graph (occupancy_grid_sig), "map")
		end

feature {NONE} -- Implementation

	occupancy_grid_sig: separate OCCUPANCY_GRID_SIGNALER
	path_pub: PATH_PUBLISHER

	graph: GRAPH_BUILDER

end -- class
