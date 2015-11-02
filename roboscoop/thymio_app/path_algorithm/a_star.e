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
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}P_optimal.make (0)
			create {ARRAYED_LIST[REAL_64]}tot_expected_cost_list.make (0)
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}Path.make (0)
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}Parent_node.make (0)
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}Parents_open.make (0)
			create {ARRAYED_LIST[SPATIAL_GRAPH_NODE]}Parents_closed.make (0)
		end

feature

	a_star_algorithm (n_start, n_goal: SPATIAL_GRAPH_NODE): PATH_MSG
		local
			n_prev, n_next, n_current: SPATIAL_GRAPH_NODE
			n_prev_neigh, n_prev_current: SPATIAL_GRAPH_NODE
			cost: COST
			cost_i: REAL_64
			i, j, k, parent_ind: INTEGER
		do
--			create n_start.make_with_coords (start_node)
--			create n_goal.make_with_coords (goal_node)
			create cost.make

			-- Input start node in S_open und cost in tot_expected_cost_list
			S_open.extend (n_start)
			tot_expected_cost_list.extend (cost.tot_expected_cost (n_start, n_start, n_start, n_goal))
			-- create SPATIAL_GRAPH_NODE.position := 0,0,0 for parent of start_node
			Parents_open.extend (create {SPATIAL_GRAPH_NODE}.make_with_coords (create {POINT_MSG}.make_empty))

			lowest_cost := {REAL_64}.positive_infinity

			debug
				io.put_string ("POS_1%N")
			end

			from
				k := 1
			until
				S_open.is_empty or S_closed.has (n_goal)
			loop
				from
					i := 1
				until
					i > S_open.count
				loop
					cost_i := cost.tot_expected_cost (n_start, S_open.at (i), S_open.at (i), n_goal)
--					cost_i := tot_expected_cost_list.at (i)

					if cost_i < lowest_cost then
						lowest_cost := cost_i
						lowest_cost_index := i
					end
					i := i + 1
				end
				n_next := S_open.at (lowest_cost_index)

--				-- save path and trim path if node was closed before
--				if not S_closed.has (n_next) then
--					Path.extend (n_next)
--				else
--					from
--						i := Path.index_of (n_next, 1)
--					until
--						Path.count = Path.index_of (n_next, 1)
--					loop
--						Path.go_i_th (i+1)
--						Path.remove
--					end
--				end

------------- VERSION WITH .go_i_th and .remove
------------------------------------------------------------------------
--				S_open.go_i_th (lowest_cost_index)
--				tot_expected_cost_list.go_i_th (lowest_cost_index)
--				Parents_open.go_i_th (lowest_cost_index)

--				S_closed.extend (n_next)
--				Parents_closed.extend (n_next)

--				S_open.remove
--				tot_expected_cost_list.remove
--				Parents_open.remove
--------------------------------------------------------------------------

				S_open.start
				tot_expected_cost_list.start
				Parents_open.start

				S_closed.extend (n_next)
				Parents_closed.extend (Parents_open.at (lowest_cost_index))

				S_open.prune_all (S_open.at (lowest_cost_index))
				tot_expected_cost_list.prune_all (tot_expected_cost_list.at (lowest_cost_index))
				Parents_open.prune_all (Parents_open.at (lowest_cost_index))
----------------------------------------------------------------------------
				debug
					io.put_string("POS_2%N"
									+ "lowest_cost: " + lowest_cost.out
									+ " lc_index: " + lowest_cost_index.out
									+ " S_open.count: " + S_open.count.out
									+ " n_next_pos: " + n_next.position.out
									+ "%N")
				end

				if n_next = n_goal then
					n_current := n_goal
					debug
						io.put_string ("POS_3" + i.out + "%N")
					end

					from
					until
						n_current.position.x = 0.0 --create {SPATIAL_GRAPH_NODE}.make_with_coords (create {POINT_MSG}.make_empty)
					loop
						P_optimal.extend (n_current)

						debug
							io.put_string ("POS_4%N"
											+ "P_optimal COUNT: " + P_optimal.count.out
											+ " n_curr_neigh_coutn: " + n_current.neighbours.count.out
											+ "%N")
						end

						if S_open.has (n_current) then
							parent_ind := S_open.index_of (n_current, 1)
							n_current := Parents_open.at (parent_ind)
						elseif S_closed.has (n_current) then
							parent_ind := S_closed.index_of (n_current, 1)
							n_current := Parents_closed.at (parent_ind)
						else
							debug
								io.put_string ("WRONG PARENT handling!")
							end
						end
					end

				else
					debug
						io.put_string ("POS_5%N")
					end

					from
						j := 1
					until
						j > n_next.neighbours.count
					loop
						if not S_open.has (n_next.neighbours.at (j)) then
							n_prev_neigh := n_next

							Parents_open.extend (n_prev_neigh)
							S_open.extend (n_next.neighbours.at (j))
							tot_expected_cost_list.extend (cost.tot_expected_cost (n_start, n_next, n_next.neighbours.at (j), n_goal))

							debug
								io.put_string ("POS_6 " + j.out + "%N")
							end
						else
							debug
								io.put_string ("POS_7a %N"
												+ "cost_neigh: " + cost.tot_expected_cost (n_start, n_next, n_next.neighbours.at (j), n_goal).out
												+ " cost_prev_neigh: " + tot_expected_cost_list.at (j).out
												+ " j: " + j.out
												+ "%N")
							end
							if cost.tot_expected_cost (n_start, n_next, n_next.neighbours.at (j), n_goal) < tot_expected_cost_list.at (j) then
								------------------------------------------------------------------------------
--								tot_expected_cost_list.start
--								Parents_open.start
								--------------------------------------------------------------------------							
								tot_expected_cost_list.go_i_th (j)
								Parents_open.go_i_th (j)
								----------------------------------------------------------------------------------
								tot_expected_cost_list.replace (cost.tot_expected_cost (n_start, n_next, n_next.neighbours.at (j), n_goal))
								n_prev_neigh := n_next
								Parents_open.replace (n_prev_neigh)
								---------------------------------------------------------------------------------
								debug
									io.put_string ("POS_7 " + j.out + "%N")
								end
							end
						end
						j := j + 1
					end
				end

				debug
					io.put_string ("LOOP: " + k.out + "%N")
				end

				k := k + 1
			end

			Result := create {PATH_MSG}.make_empty -- only for testing

			debug
				io.put_string ("END!!!%N"
								+ " S_open count: " + S_open.count.out
								+ "%N")
			end

		end

feature

	S_closed: LIST[SPATIAL_GRAPH_NODE]
	S_open: LIST[SPATIAL_GRAPH_NODE]
	lowest_cost: REAL_64
	lowest_cost_index: INTEGER
	tot_expected_cost_list: LIST[REAL_64]
	P_optimal: LIST[SPATIAL_GRAPH_NODE]
	Path: LIST[SPATIAL_GRAPH_NODE]
	Parent_node: LIST[SPATIAL_GRAPH_NODE]
	Parents_open: LIST [SPATIAL_GRAPH_NODE]
	Parents_closed: LIST [SPATIAL_GRAPH_NODE]

end -- class
