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
			create {ARRAYED_LIST[REAL_64]}g_cost.make (0)
			create {ARRAYED_LIST[REAL_64]}f_cost.make (0)
		end

feature

	a_star_algorithm (n_start, n_goal: SPATIAL_GRAPH_NODE): ARRAYED_LIST[SPATIAL_GRAPH_NODE]
		local
			n_prev, n_next, n_current: SPATIAL_GRAPH_NODE
			n_prev_neigh, n_prev_current: SPATIAL_GRAPH_NODE
			cost: COST
			cost_i, g_cost_temp: REAL_64
			i, j, k, l, parent_ind: INTEGER
		do
			create cost.make

			g_cost.extend (0) -- path cost for n_start
			f_cost.extend (g_cost.at (1) + cost.cost (n_start, n_goal)) -- total expected cost for n_start

			-- Input start node in S_open and cost in tot_expected_cost_list
			S_open.extend (n_start)
			tot_expected_cost_list.extend (cost.tot_expected_cost (n_start, n_start, n_start, n_goal))
			-- create SPATIAL_GRAPH_NODE.position := 0,0,0 for parent of start_node
			Parents_open.extend (create {SPATIAL_GRAPH_NODE}.make_with_coords (create {POINT_MSG}.make_empty))

			debug
				io.put_string ("POS_1%N")
			end

			from
				k := 1
			until
				S_open.is_empty or S_closed.has (n_goal)
			loop
				debug
					io.put_string ("%NLOOP: " + k.out + "%N")
				end

				lowest_cost := {REAL_64}.positive_infinity

				from
					i := 1
				until
					i > S_open.count
				loop
					cost_i := f_cost.at (i)
--					cost.tot_expected_cost (n_start, S_open.at (i), S_open.at (i), n_goal)
--					cost_i := tot_expected_cost_list.at (i)

					if cost_i < lowest_cost then
						lowest_cost := cost_i
						lowest_cost_index := i
					end
					i := i + 1
				end
				n_next := S_open.at (lowest_cost_index)

--				S_open.start
----				tot_expected_cost_list.start
--				Parents_open.start
--				g_cost.start
--				f_cost.start

--				S_closed.extend (n_next)
--				Parents_closed.extend (Parents_open.at (lowest_cost_index))

--				S_open.prune (S_open.at (lowest_cost_index))
----				tot_expected_cost_list.prune_all (tot_expected_cost_list.at (lowest_cost_index))
--				g_cost.prune (g_cost.at (lowest_cost_index))
--				f_cost.prune (f_cost.at (lowest_cost_index))
--				Parents_open.prune (Parents_open.at (lowest_cost_index))

				debug
					from
						l := 1
					until
						l > S_open.count
					loop
						io.put_string ("POS_2_1 | "
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
					io.put_string("POS_2_2 | "
									+ "lowest_cost: " + lowest_cost.out + " | "
									+ "lc_index: " + lowest_cost_index.out + " | "
									+ "S_open.count: " + S_open.count.out + "%N"
									+ "n_next_pos: " + n_next.position.out
									+ "%N")
				end

				if n_next = n_goal then
					n_current := n_goal
					P_optimal.extend (n_current)

					debug
						io.put_string ("POS_3%N")
					end

					from
					until
--						n_current.position.x = 0.0 --create {SPATIAL_GRAPH_NODE}.make_with_coords (create {POINT_MSG}.make_empty)
						n_current = n_start
					loop
						debug
							io.put_string ("POS_4 | "
											+ "P_optimal COUNT: " + P_optimal.count.out + " | "
											+ "n_curr_neigh_count: " + n_current.neighbours.count.out + "%N"
											+ "P_optimal_path_pos: " + P_optimal.last.position.out
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
						P_optimal.extend (n_current)
					end

				else
					debug
						io.put_string ("POS_5 | S_open.index_of (n_next, 1): "  + S_open.index_of (n_next, 1).out
										+ "%N" )
					end

					from
						j := 1
					until
						j > n_next.neighbours.count
					loop
						if not S_open.has (n_next.neighbours.at (j)) then
							g_cost.extend (g_cost.at (S_open.index_of (n_next, 1)) + cost.cost (n_next.neighbours.at (j), n_next))
							f_cost.extend (g_cost.last + cost.cost (n_next.neighbours.at (j), n_goal))

							S_open.extend (n_next.neighbours.at (j))
--							Parents_open.extend (n_next.neighbours.at (j))
							Parents_open.extend (n_next)
							n_prev_neigh := n_next

--							tot_expected_cost_list.extend (cost.tot_expected_cost (n_start, n_next, n_next.neighbours.at (j), n_goal))
							debug
								io.put_string ("POS_6_" + j.out + " | "
												+ "g_cost.count: " + g_cost.count.out
												+ "%N")
							end

						else
							g_cost_temp := g_cost.at (S_open.index_of (n_next, 1)) + cost.cost (n_next.neighbours.at (j), n_next)

							debug
								io.put_string ("POS_7 | "
--												+ "cost_neigh: " + cost.tot_expected_cost (n_start, n_next, n_next.neighbours.at (j), n_goal).out + " | "
--												+ "cost_prev_neigh: " + tot_expected_cost_list.at (j).out + " | "
												+ "cost_neigh: " + g_cost_temp.out + " | "
												+ "cost_prev_neigh: " + g_cost.at (S_open.index_of (n_next.neighbours.at (j), 1)).out + " | "
												+ "j: " + j.out
												+ "%N")
							end
							if g_cost_temp < g_cost.at (S_open.index_of (n_next.neighbours.at (j), 1)) then
--							if cost.tot_expected_cost (n_start, n_next, n_next.neighbours.at (j), n_goal) < tot_expected_cost_list.at (j) then
								g_cost.put_i_th (g_cost_temp, S_open.index_of (n_next.neighbours.at (j), 1))
								f_cost.put_i_th (g_cost.at (S_open.index_of (n_next.neighbours.at (j), 1)) + cost.cost (n_next.neighbours.at (j), n_goal),
																S_open.index_of (n_next.neighbours.at (j),1))

								------------------------------------------------------------------------------
--								tot_expected_cost_list.start
--								Parents_open.start
								--------------------------------------------------------------------------							
--								tot_expected_cost_list.go_i_th (j)
--								Parents_open.go_i_th (j)
								----------------------------------------------------------------------------------
--								tot_expected_cost_list.replace (cost.tot_expected_cost (n_start, n_next, n_next.neighbours.at (j), n_goal))
								Parents_open.put_i_th (n_next, S_open.index_of (n_next.neighbours.at (j),1))
								n_prev_neigh := n_next
--								Parents_open.replace (n_prev_neigh)
								---------------------------------------------------------------------------------
								debug
									io.put_string ("POS_8_" + j.out + "%N")
								end
							end
						end
						j := j + 1
					end
				end

				S_open.start
--				tot_expected_cost_list.start
				Parents_open.start
				g_cost.start
				f_cost.start

				S_closed.extend (n_next)
				Parents_closed.extend (Parents_open.at (lowest_cost_index))
--				Parents_closed.extend (Parents_open.at (S_open.index_of (n_next, 1)))

--				S_open.prune (S_open.at (lowest_cost_index))
				S_open.prune (n_next)

--				tot_expected_cost_list.prune_all (tot_expected_cost_list.at (lowest_cost_index))
				g_cost.prune (g_cost.at (lowest_cost_index))
				f_cost.prune (f_cost.at (lowest_cost_index))

				Parents_open.prune (Parents_open.at (lowest_cost_index))
--				Parents_open.prune (Parents_open.at (S_open.index_of (n_next, 1)))

				k := k + 1
			end

			create Result.make (0)
			Result := P_optimal

			debug
				io.put_string ("END!!!%N"
								+ " S_open count: " + S_open.count.out + " | "
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
		end

feature

	S_closed: LIST[SPATIAL_GRAPH_NODE]
	S_open: LIST[SPATIAL_GRAPH_NODE]
	lowest_cost: REAL_64
	lowest_cost_index: INTEGER
	tot_expected_cost_list: LIST[REAL_64]
	P_optimal: ARRAYED_LIST[SPATIAL_GRAPH_NODE]
	Path: LIST[SPATIAL_GRAPH_NODE]
	Parent_node: LIST[SPATIAL_GRAPH_NODE]
	Parents_open: LIST [SPATIAL_GRAPH_NODE]
	Parents_closed: LIST [SPATIAL_GRAPH_NODE]
	g_cost, f_cost: LIST[REAL_64]

end -- class
