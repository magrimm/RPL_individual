note
	description: "Graph builder class."
	author: "Maris Grimm"
	date: "30.10.15"

class
	GRAPH_BUILDER

feature -- Access

	grid_to_graph (occ_grid_sig: separate OCCUPANCY_GRID_SIGNALER): ARRAYED_LIST[SPATIAL_GRAPH_NODE]
		-- Convert grid to graph
		require
			-- Make sure that map is received
			occ_grid_sig.state.data.count > 0
		local
--			conn_strategy: FOUR_CONNECTED_PATH		                                             --------------------------------------- TODO PARAM
			conn_strategy: EIGHT_CONNECTED_PATH
			grid_graph: GRID_GRAPH
			a_star: SEARCH_ALGORITHM

			min_x, min_y, max_x, max_y, resolution: REAL_64
			start_node_ind_x, start_node_ind_y, goal_node_ind_x, goal_node_ind_y, i, j: INTEGER

			start_node, goal_node: POINT_MSG
			n_start, n_goal: SPATIAL_GRAPH_NODE

		do
			create conn_strategy
			create a_star.make

			-- Calculate map in meters
			min_x := occ_grid_sig.state.info.origin.position.x
			min_y := occ_grid_sig.state.info.origin.position.y
			max_x := occ_grid_sig.state.info.origin.position.x +
						occ_grid_sig.state.info.resolution * occ_grid_sig.state.info.width
			max_y := occ_grid_sig.state.info.origin.position.y +
						occ_grid_sig.state.info.resolution * occ_grid_sig.state.info.height

			-- Create 2d graph from grid
			create grid_graph.make_2d (occ_grid_sig.state.info.width.as_integer_32,
										occ_grid_sig.state.info.height.as_integer_32,
										min_x, max_x, min_y, max_y,
										conn_strategy)

			-- Inflate obstacles to assume point mass
			occ_grid_sig.inflate (0.00)                                                           -------------------------------------- TODO PARAM

			-- Remove connections at obstacles
			from
				i := 1
			until
				i > grid_graph.count_x
			loop
				from
					j := 1
				until
					j > grid_graph.count_y
				loop
					if occ_grid_sig.occupancy (i, j) > occ_grid_sig.occupancy_threshold then
						grid_graph.add_obstacle_by_index (i, j, 1)
					end
					j := j + 1
				end
				i := i +1
			end

			-- Input x,y,z coord and get out POINT_MSG
			create start_node.make_with_values (0.2, 0.2, 0)                                        -------------------------------------- TODO PARAM
			create goal_node.make_with_values (0.5, 0.5, 0)                                         -------------------------------------- TODO PARAM

			-- Input POINT_MSG and get out SPATIAL_GRAPH_NODE
			create n_start.make_with_coords (create {POINT_MSG}.make_empty)
			create n_goal.make_with_coords (create {POINT_MSG}.make_empty)

			resolution := occ_grid_sig.state.info.resolution

			-- Calculation node indices from POINT_MSG
			start_node_ind_x := (start_node.x/resolution).floor
			start_node_ind_y := (start_node.y/resolution).floor
			goal_node_ind_x := (goal_node.x/resolution).floor
			goal_node_ind_y := (goal_node.y/resolution).floor

			-- Input indices and get out SPATIAL_GRAPH_NODE (.node_at...)
			n_start := grid_graph.node_at (start_node_ind_x, start_node_ind_y, 1)
			n_goal := grid_graph.node_at (goal_node_ind_x, goal_node_ind_y, 1)

			Result := a_star.a_star_algorithm (n_start, n_goal)
		end

end -- class
