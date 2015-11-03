note
	description: "Summary description for {A_STAR}."
	author: "Marius Grimm"
	date: "30.10.15"

class
	A_STAR

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

			create path_publisher_S_open.make_with_topic ("/S_open")
			create path_publisher_S_closed.make_with_topic ("/S_closed")
		end

feature -- Access

	a_star_algorithm (n_start, n_goal: SPATIAL_GRAPH_NODE): ARRAYED_LIST[SPATIAL_GRAPH_NODE]
		local
			n_next, n_current: SPATIAL_GRAPH_NODE
			cost: COST
			cost_i, g_cost_temp, g_cost_n_next, f_cost_n_next: REAL_64
			i, j, k, l, parent_ind: INTEGER
		do
			create cost.make

			-- Input start node in S_open and save costs for start node
			S_open.extend (n_start)
			g_cost.extend (0) 														-- path cost for n_start
			f_cost.extend (g_cost.at (1) + cost.cost (n_start, n_goal)) 			-- total expected cost for n_start

			-- create SPATIAL_GRAPH_NODE.position := 0,0,0 for parent of start_node
			Parents_open.extend (create {SPATIAL_GRAPH_NODE}.make_with_coords (create {POINT_MSG}.make_empty))

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

					if cost_i < lowest_cost then
						lowest_cost := cost_i
						lowest_cost_index := i
					end
					i := i + 1
				end
				n_next := S_open.at (lowest_cost_index)
				g_cost_n_next := g_cost.at (lowest_cost_index)
				f_cost_n_next := f_cost.at (lowest_cost_index)

				S_open.start
				Parents_open.start
				g_cost.start
				f_cost.start

				S_closed.extend (n_next)
				Parents_closed.extend (Parents_open.at (lowest_cost_index))

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
					n_current := n_goal
					P_optimal.extend (n_current)

					from
					until
--						n_current.position.x = 0.0
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
							parent_ind := S_open.index_of (n_current, 1)
							n_current := Parents_open.at (parent_ind)
						elseif S_closed.has (n_current) then
							parent_ind := S_closed.index_of (n_current, 1)
							n_current := Parents_closed.at (parent_ind)
						else
							debug
								io.put_string ("ERROR: wrong PARENT handling!")
							end
						end
						P_optimal.extend (n_current)
					end

				else
					debug
						io.put_string ("POS_3: "
										+ "%N" )
					end

					from
						j := 1
					until
						j > n_next.neighbours.count
					loop
						if not S_closed.has (n_next.neighbours.at (j)) then
							if not S_open.has (n_next.neighbours.at (j)) then
--								g_cost.extend (g_cost.at (S_open.index_of (n_next, 1)) + cost.cost (n_next.neighbours.at (j), n_next))
--								f_cost.extend (g_cost.last + cost.cost (n_next.neighbours.at (j), n_goal))
								debug
									io.put_string ("low_ind: " + lowest_cost_index.out + " | "
													+ "g_count: " + g_cost.count.out)
								end

								g_cost.extend (g_cost_n_next + cost.cost (n_next.neighbours.at (j), n_next))

--								g_cost.extend (g_cost.at (lowest_cost_index) + cost.cost (n_next.neighbours.at (j), n_next))
								f_cost.extend (g_cost.at (g_cost.count) + cost.cost (n_next.neighbours.at (j), n_goal))

								S_open.extend (n_next.neighbours.at (j))
								Parents_open.extend (n_next)

								debug
									io.put_string ("POS_4_" + j.out + " | "
													+ "g_cost.count: " + g_cost.count.out
													+ "%N")
								end

							else
--								g_cost_temp := g_cost.at (S_open.index_of (n_next, 1)) + cost.cost (n_next.neighbours.at (j), n_next)
--								g_cost_temp := g_cost.at (lowest_cost_index) + cost.cost (n_next.neighbours.at (j), n_next)
								g_cost_temp := g_cost_n_next + cost.cost (n_next.neighbours.at (j), n_next)


								debug
									io.put_string ("POS_5 | "
													+ "cost_neigh: " + g_cost_temp.out + " | "
													+ "cost_prev_neigh: " + g_cost.at (S_open.index_of (n_next.neighbours.at (j), 1)).out + " | "
													+ "j: " + j.out
													+ "%N")
								end
								if g_cost_temp < g_cost.at (S_open.index_of (n_next.neighbours.at (j), 1)) then
									g_cost.put_i_th (g_cost_temp, S_open.index_of (n_next.neighbours.at (j), 1))
--									f_cost.put_i_th (g_cost.at (S_open.index_of (n_next.neighbours.at (j), 1)) + cost.cost (n_next.neighbours.at (j), n_goal),
--																	S_open.index_of (n_next.neighbours.at (j),1))
									f_cost.put_i_th (g_cost_temp + cost.cost (n_next.neighbours.at (j), n_goal),
																	S_open.index_of (n_next.neighbours.at (j),1))

									Parents_open.put_i_th (n_next, S_open.index_of (n_next.neighbours.at (j),1))
								end
							end
						end
						j := j + 1
					end
				end

--				S_open.start
--				Parents_open.start
--				g_cost.start
--				f_cost.start

--				S_closed.extend (n_next)
--				Parents_closed.extend (Parents_open.at (lowest_cost_index))
----				Parents_closed.extend (Parents_open.at (S_open.index_of (n_next, 1)))

----				S_open.prune (S_open.at (lowest_cost_index))
--				S_open.prune (n_next)

--				g_cost.prune (g_cost.at (lowest_cost_index))
--				f_cost.prune (f_cost.at (lowest_cost_index))

--				Parents_open.prune (Parents_open.at (lowest_cost_index))
----				Parents_open.prune (Parents_open.at (S_open.index_of (n_next, 1)))

				path_publisher_S_open.set_path_with_spatial_graph_nodes (S_open)
				path_publisher_S_closed.set_path_with_spatial_graph_nodes (S_closed)

				k := k + 1
			end

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
