note
	description: "Summary description for {GRID_TO_GRAPH}."
	author: "Maris Grimm"
	date: "30.10.15"

class
	GRID_TO_GRAPH

feature

	grid_to_graph (occ_grid_sig: separate OCCUPANCY_GRID_SIGNALER): ARRAYED_LIST[SPATIAL_GRAPH_NODE]
		-- convert grid to graph
		require
			-- make sure that map is received
			occ_grid_sig.state.data.count > 0
		local
			grid_graph: GRID_GRAPH
			min_x, min_y, max_x, max_y: REAL_64
			conn_strategy: FOUR_CONNECTED_PATH
			i, j: INTEGER
			n_start, n_goal: SPATIAL_GRAPH_NODE

			a_star: A_STAR
			start_node, goal_node: POINT_MSG
			start_n, goal_n: SPATIAL_GRAPH_NODE
			start_node_ind_x, start_node_ind_y, goal_node_ind_x, goal_node_ind_y: INTEGER
			resolution: REAL_64
--			path: PATH_MSG

		do
			create conn_strategy
			-- Calculate map in meters
			debug
				io.put_string ("width: " + occ_grid_sig.state.info.width.as_integer_32.out
								+ " count: " + occ_grid_sig.state.data.count.out
								+ "%N")
			end

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



			create a_star.make

			-- Input x,y,z coord and gt out POINT_MSG
			create start_node.make_with_values (0.2, 0.2, 0)
			create goal_node.make_with_values (0.3, 0.2, 0)

			-- Input POINT_MSG and get out SPATIAL_GRAPH_NODE
			create start_n.make_with_coords (create {POINT_MSG}.make_with_values (0, 0, 0))
			create goal_n.make_with_coords (create {POINT_MSG}.make_with_values (0, 0, 0))

			resolution := occ_grid_sig.state.info.resolution

			start_node_ind_x := (start_node.x/resolution).floor
			start_node_ind_y := (start_node.y/resolution).floor
			goal_node_ind_x := (goal_node.x/resolution).floor
			goal_node_ind_y := (goal_node.y/resolution).floor

			-- input indices and get out SPATIAL_GRAPH_NODE (.node_at...)
			start_n := grid_graph.node_at (start_node_ind_x, start_node_ind_y, 1)
			goal_n := grid_graph.node_at (goal_node_ind_x, goal_node_ind_y, 1)

			Result := a_star.a_star_algorithm (start_n, goal_n)
		end

end -- class
