note
	description: "A_star optimal path search algorithm."
	author: "Marius Grimm"
	date: "30.10.15"

class
	SEARCH_ALGORITHM

create
	make

feature {NONE}

	make
		do
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}S_closed.make (0)
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}S_open.make (0)
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}P_optimal.make (0)
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}Parents_open.make (0)
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}Parents_closed.make (0)
			create {ARRAYED_LIST[REAL_64]}g_cost.make (0)
			create {ARRAYED_LIST[REAL_64]}f_cost.make (0)

			create path_publisher_S_open.make_with_topic ({MAP_TOPICS}.s_open)
			create path_publisher_S_closed.make_with_topic ({MAP_TOPICS}.s_closed)
		end

feature -- Access

	a_star_algorithm (n_start, n_goal: SPATIAL_GRAPH_NODE): ARRAYED_LIST[SPATIAL_GRAPH_NODE]
		local
			n_next, n_current: SPATIAL_GRAPH_NODE
			cost: COST
			cost_i, g_cost_temp, g_cost_n_next, f_cost_n_next: REAL_64
			i, j, k, l, parent_ind: INTEGER
		do
			-- create heuristic cost
			create cost.make

			-- Input start node in S_open (queue of nodes to visit) and save costs for start node
			S_open.extend (n_start)
			-- Path cost and total expected cost arrays for all nodes in S_open
			g_cost.extend (0) 														-- path cost for start node
			f_cost.extend (g_cost.at (1) + cost.cost (n_start, n_goal)) 			-- total expected cost for start node

			-- create SPATIAL_GRAPH_NODE.position := 0,0,0 for parent of start_node
			Parents_open.extend (create {SPATIAL_GRAPH_NODE}.make_with_coords (create {POINT_MSG}.make_empty))

			from
				k := 1
			until
				-- All nodes visited or goal reached
				S_open.is_empty or S_closed.has (n_goal)
			loop
				lowest_cost := {REAL_64}.positive_infinity

				from
					i := 1
				until
					i > S_open.count
				loop
					-- Get index and cost of lowest cost in S_open
					cost_i := f_cost.at (i)

					if cost_i < lowest_cost then
						lowest_cost := cost_i
						lowest_cost_index := i
					end
					i := i + 1
				end
				-- n_next is next node to visit with the lowest cost of all nodes in S_open
				n_next := S_open.at (lowest_cost_index)
				-- path cost and total expected cost of next node to visit
				g_cost_n_next := g_cost.at (lowest_cost_index)
				f_cost_n_next := f_cost.at (lowest_cost_index)

				-- Move cursor positions to start
				S_open.start
				Parents_open.start
				g_cost.start
				f_cost.start

				-- Add node n_next to set of visited nodes in S_closed and save its parent node in Parents_closed
				S_closed.extend (n_next)
				Parents_closed.extend (Parents_open.at (lowest_cost_index))

				-- Remove n_next from S_open, remove path cost and total expected cost at
				-- same index position and remove parent node from Parents_open
				S_open.prune (S_open.at (lowest_cost_index))
				g_cost.prune (g_cost.at (lowest_cost_index))
				f_cost.prune (f_cost.at (lowest_cost_index))
				Parents_open.prune (Parents_open.at (lowest_cost_index))

				debug
					from
						l := 1
					until
						l > S_open.count
					loop
						io.put_string ("POS_1_1 | "
										+ "S_open_pos_" + l.out + " : " + S_open.at (l).position.out
										+ "g_cost: " + g_cost.at (l).out + " | "
										+ "f_cost: " + f_cost.at (l).out
										+ "%N%NParent_open_pos_" + l.out + " : " + Parents_open.at (l).position.out
										+ "%N")

						l := l + 1
					end
					from
						l := 1
					until
						l > S_closed.count
					loop
						io.put_string ("S_closed_pos_" + l.out + " : " + S_closed.at (l).position.out
										+ "%NParent_closed_pos_" + l.out + " : " + Parents_closed.at (l).position.out
										+ "%N")
						l := l + 1
					end
					io.put_string("POS_1_2 | "
									+ "lowest_cost: " + lowest_cost.out + " | "
									+ "lc_index: " + lowest_cost_index.out + " | "
									+ "S_open.count: " + S_open.count.out + "%N"
									+ "n_next_pos: " + n_next.position.out
									+ "%N")
				end

				if n_next = n_goal then
					-- If goal reached execute
					n_current := n_goal
					P_optimal.extend (n_current)

					from
					until
						-- Optimal path P_optimal traced back to start node
						n_current = n_start
					loop
						debug
							io.put_string ("POS_2 | "
											+ "P_optimal COUNT: " + P_optimal.count.out + " | "
											+ "n_curr_neigh_count: " + n_current.neighbours.count.out + "%N"
											+ "P_optimal_path_pos: " + P_optimal.last.position.out
											+ "%N")
						end

						if S_open.has (n_current) then
							-- If current node of optimal path is in S_open
							parent_ind := S_open.index_of (n_current, 1)
							n_current := Parents_open.at (parent_ind)
						elseif S_closed.has (n_current) then
							-- If current node of optimal path is in S_closed
							parent_ind := S_closed.index_of (n_current, 1)
							n_current := Parents_closed.at (parent_ind)
						else
							-- Error handling
							io.put_string ("ERROR: wrong PARENT handling!")
						end
						-- Add current node to optimal path
						P_optimal.extend (n_current)
					end

				else
					-- Goal not reached
					from
						j := 1
					until
						j > n_next.neighbours.count
					loop
						if not S_closed.has (n_next.neighbours.at (j)) then
							-- If neighbour node j of node n_next was not already visited before
							if not S_open.has (n_next.neighbours.at (j)) then
								-- If neighbour node j of node n_next is not yet in queue of nodes to visit
								debug
									io.put_string ("POS_3 | low_ind: " + lowest_cost_index.out + " | "
													+ "g_count: " + g_cost.count.out)
								end

								-- Add path cost and total expected cost of neighbour node j of node n_next to g_cost and f_cost
								g_cost.extend (g_cost_n_next + cost.cost (n_next.neighbours.at (j), n_next))
								f_cost.extend (g_cost.at (g_cost.count) + cost.cost (n_next.neighbours.at (j), n_goal))
								-- Add neighbour j to queue of nodes to visit and add the parent node (n_next) to Parents_open
								S_open.extend (n_next.neighbours.at (j))
								Parents_open.extend (n_next)

							else
								-- If neighbour node j of node n_next is already in the queue of nodes to visit
								g_cost_temp := g_cost_n_next + cost.cost (n_next.neighbours.at (j), n_next)

								debug
									io.put_string ("POS_4 | "
													+ "cost_neigh: " + g_cost_temp.out + " | "
													+ "cost_prev_neigh: " + g_cost.at (S_open.index_of (n_next.neighbours.at (j), 1)).out + " | "
													+ "j: " + j.out
													+ "%N")
								end

								if g_cost_temp < g_cost.at (S_open.index_of (n_next.neighbours.at (j), 1)) then
									-- Check wheather the path cost of the neighbour j is now lower than before
									-- Replace path cost, total expected cost and parent node of neighbour j with new values
									g_cost.put_i_th (g_cost_temp, S_open.index_of (n_next.neighbours.at (j), 1))
									f_cost.put_i_th (g_cost_temp + cost.cost (n_next.neighbours.at (j), n_goal),
																	S_open.index_of (n_next.neighbours.at (j),1))

									Parents_open.put_i_th (n_next, S_open.index_of (n_next.neighbours.at (j),1))
								end
							end
						end
						j := j + 1
					end
				end

				-- Publish the evolution of the nodes in the queue to visit and the visited nodes
				path_publisher_S_open.set_path_with_spatial_graph_nodes (S_open, "map")
				path_publisher_S_closed.set_path_with_spatial_graph_nodes (S_closed, "map")

				k := k + 1
			end -- Loop until all nodes visited or goal reached

			debug
				io.put_string ("END!!!%N"
								+ "P_optimal.count: " + P_optimal.count.out
								+ "%N")
				from
					l := 1
				until
					l > P_optimal.count
				loop
					io.put_string ("P_optimal_pos_" + l.out + ": " + P_optimal.at (l).position.out
									+ "%N")
					l := l + 1
				end
			end

			-- Create result and indicate state
			create Result.make (0)
			if S_closed.has (n_goal) then
				io.put_string ("GOAL REACHED!")
				Result := P_optimal
			elseif S_open.is_empty then
				io.put_string ("GOAL NOT REACHABLE!")
			end
		end

feature {NONE}

	Parents_open, Parents_closed: LIST[SPATIAL_GRAPH_NODE]
	g_cost, f_cost: LIST[REAL_64]
	S_open, S_closed, P_optimal: ARRAYED_LIST[SPATIAL_GRAPH_NODE]
	lowest_cost: REAL_64
	lowest_cost_index: INTEGER

	path_publisher_S_open, path_publisher_S_closed: PATH_PUBLISHER

end -- class
