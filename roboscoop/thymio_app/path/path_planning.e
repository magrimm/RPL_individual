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

	test
		local
			point_1, point_2: POINT_MSG
			sgn_1, sgn_2: SPATIAL_GRAPH_NODE
			a_sgn: ARRAYED_LIST[SPATIAL_GRAPH_NODE]
		do
			-- create POINT_MSG
			create point_1.make_with_values (0, 0, 0)
			create point_2.make_with_values (0.5, 0.2, 0)

			-- create SPATIAL_GRAPH_NODE
			create sgn_1.make_with_coords (point_1)
			create sgn_2.make_with_coords (point_2)

			-- create ARRAYED_LIST [SPATIAL_GRAPH_NODE]
			create a_sgn.make (0)

			-- fill ARRAYED_LIST with SPATIAL_GRAPH_NODE
			a_sgn.force (sgn_1)
			a_sgn.force (sgn_2)

			-- publish path
			path_planner.set_path_with_spatial_graph_nodes (a_sgn)
		end

	plan_path
		local
--			n_start, n_goal: POINT_MSG
			grid_to_graph: GRID_TO_GRAPH

		do
			create grid_to_graph

--			-- create start and goal node with coordinates
--			create n_start.make_with_values (20, 20, 0)
--			create n_goal.make_with_values (20, 180, 0)

			-- publish path
			path_planner.set_path_with_spatial_graph_nodes (grid_to_graph.grid_to_graph (occupancy_grid_sig))
		end

feature {NONE} -- Implementation

	occupancy_grid_sig: separate OCCUPANCY_GRID_SIGNALER
	path_planner: PATH_PUBLISHER

end -- class
