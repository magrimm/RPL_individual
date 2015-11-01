note
	description: "Summary description for {FOUR_CONNECTED_PATH}."
	author: "Marius Grimm"
	date: "30.10.15"

class
	FOUR_CONNECTED_PATH

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
					j := j + 1
				end
				i := i + 1
			end
			debug
				io.put_string ("g.count_x: " + g.count_x.out
								+ "g.nodes.at (100): "
								 + "%N")

				across g.nodes as ic loop
					io.put_string ("item.pos: " + ic.item.position.out + "%N")
					io.put_string ("item.nei: " + ic.item.neighbours.out + "%N")
				end
			end
		end
end -- class
