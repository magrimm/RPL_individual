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
		local
			i, j: INTEGER
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
					connect_node (g, i, j, 1, i+1, j, 1)
					connect_node (g, i, j, 1, i, j+1, 1)
					connect_node (g, i, j, 1, i-1, j, 1)
					connect_node (g, i, j, 1, i, j-1, 1)

					connect_node (g, i, j, 1, i+1, j+1, 1)
					connect_node (g, i, j, 1, i+1, j-1, 1)
					connect_node (g, i, j, 1, i-1, j+1, 1)
					connect_node (g, i, j, 1, i-1, j-1, 1)

					j := j + 1
				end
				i := i + 1
			end
		end

end -- class
