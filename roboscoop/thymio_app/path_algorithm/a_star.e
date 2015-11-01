note
	description: "Summary description for {A_STAR}."
	author: "Marius Grimm"
	date: "30.10.15"

class
	A_STAR

create
	make

feature

	make
		do
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}S_closed.make (0)
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}S_open.make (0)
		end

feature

	a_star_algorithm (start_node, goal_node: POINT_MSG): PATH_MSG
		local
			n_start, n_goal: SPATIAL_GRAPH_NODE
			g_cost, f_cost, h_cost, c_cost: REAL_64
			n_prev, n_next, n_current: SPATIAL_GRAPH_NODE
			n_prev_neigh: SPATIAL_GRAPH_NODE
			cost: COST
			cost_i: REAL_64
			i: INTEGER
		do
			create n_start.make_with_coords (start_node)
			create n_goal.make_with_coords (goal_node)

			debug
				io.put_string ("n_start: " + n_start.position.x.out
								+ " n_goal: " + n_goal.out
								+ "%N")
			end

			S_open.extend (n_start)
			g_cost := 0
			f_cost := g_cost + h_cost


			lowest_cost := {REAL_64}.positive_infinity
			create cost.make
			from
			until
--				S_open.is_empty or (goal_pos = S_closed.last) -- if priority queue
				S_open.is_empty or S_closed.has (n_goal)
			loop
				from
					i := 1
				until
					i > S_open.count
				loop
					debug
						io.put_string ("S_closed.at (1): " + S_open.at (1).out + "%N")
					end
					cost_i := cost.tot_expected_cost (n_start, S_open.at (i), S_open.at (i), n_goal)

					if cost_i < lowest_cost then
						lowest_cost := cost_i
						lowest_cost_index := i
					end
					i := i + 1
				end
				n_next := S_open.at (lowest_cost_index)
				S_open.go_i_th (lowest_cost_index)
				S_closed.extend (n_next)
				S_open.remove
				debug
					io.put_string("lowest_cost: " + lowest_cost.out
									+ " lc_index: " + lowest_cost_index.out
									+ " count: " + S_open.count.out
									+ "%N")
				end

				if n_next = n_goal then
					n_current := n_goal
					from
					until

					loop

					end
				else
					from
						i := 1
					until
						i > n_next.neighbours.count
					loop
						if not S_open.has (n_next.neighbours.at (i)) then
							n_prev_neigh := n_next
							S_open.extend (n_next.neighbours.at (i))
						else

						end
					end
				end
			end

--			g.nodes.item.position -- access position of graph
--			g.nodes.item.neighbours -- access connections of graph

			Result := create {PATH_MSG}.make_empty -- only for testing
		end

feature

	S_closed: LIST[SPATIAL_GRAPH_NODE]
	S_open: LIST[SPATIAL_GRAPH_NODE]
	lowest_cost: REAL_64
	lowest_cost_index: INTEGER

end -- class
