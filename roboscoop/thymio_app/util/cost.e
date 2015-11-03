note
	description: "Cost class for graph of nodes."
	author: "Marius Grimm"
	date: "01.11.14"

class
	COST

inherit
	COST_HEURISTIC

create
	make

feature {NONE}
	make
		do
			create tm
		end

feature -- access

	cost (a, b: separate SPATIAL_GRAPH_NODE): REAL_64
			-- Heuristic cost between two nodes.
		do
			Result := tm.euclidean_distance (a.position, b.position)
		end

feature {NONE}

	tm: TRIGONOMETRY_MATH

end -- class
