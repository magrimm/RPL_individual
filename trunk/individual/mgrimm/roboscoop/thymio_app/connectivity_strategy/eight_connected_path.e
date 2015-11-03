note
	description: "Connectivity strategy with eight-connected-path."
	author: "Marius Grimm"
	date: "30.10.15"

class
	EIGHT_CONNECTED_PATH

inherit
	GRID_CONNECTIVITY_STRATEGY

feature

	connect (g: separate GRID_GRAPH)
		-- connect each node in a 3D graph with all of its 26 direct neighbours.
		local
			i, j, k: INTEGER
		do
			from
				i := 1
			until
				i > g.count_x
			loop
				from
					j := 1
				until
					j > g.count_y
				loop
					from
						k := 1
					until
						k > g.count_z
					loop
						-- nine neighbours for k-1
						connect_node (g, i, j, k, i+1, j, k-1)
						connect_node (g, i, j, k, i, j+1, k-1)
						connect_node (g, i, j, k, i-1, j, k-1)
						connect_node (g, i, j, k, i, j-1, k-1)
						connect_node (g, i, j, k, i+1, j+1, k-1)
						connect_node (g, i, j, k, i+1, j-1, k-1)
						connect_node (g, i, j, k, i-1, j+1, k-1)
						connect_node (g, i, j, k, i-1, j-1, k-1)
						connect_node (g, i, j, k, i, j, k-1)

						-- eight neighbours for k
						connect_node (g, i, j, k, i+1, j, k)
						connect_node (g, i, j, k, i, j+1, k)
						connect_node (g, i, j, k, i-1, j, k)
						connect_node (g, i, j, k, i, j-1, k)
						connect_node (g, i, j, k, i+1, j+1, k)
						connect_node (g, i, j, k, i+1, j-1, k)
						connect_node (g, i, j, k, i-1, j+1, k)
						connect_node (g, i, j, k, i-1, j-1, k)

						-- nine neighbours for k+1
						connect_node (g, i, j, k, i+1, j, k+1)
						connect_node (g, i, j, k, i, j+1, k+1)
						connect_node (g, i, j, k, i-1, j, k+1)
						connect_node (g, i, j, k, i, j-1, k+1)
						connect_node (g, i, j, k, i+1, j+1, k+1)
						connect_node (g, i, j, k, i+1, j-1, k+1)
						connect_node (g, i, j, k, i-1, j+1, k+1)
						connect_node (g, i, j, k, i-1, j-1, k+1)
						connect_node (g, i, j, k, i, j, k+1)

						k := k + 1
					end
					j := j + 1
				end
				i := i + 1
			end
		end

end -- class
