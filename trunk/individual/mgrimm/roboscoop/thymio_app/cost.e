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
	-- a: starting node, b: current node, c: next node, d: goal node

	cost (c, d: separate SPATIAL_GRAPH_NODE): REAL_64
			-- Heuristic cost between two nodes.
		do
			Result := tm.euclidean_distance (c.position, d.position)
		end

	path_cost (a, b, c: separate SPATIAL_GRAPH_NODE): REAL_64
			-- Path cost from starting node a to node b
		do
			Result := tm.euclidean_distance (a.position, b.position) + traversal_cost (b, c)
		end

	traversal_cost (b, c: separate SPATIAL_GRAPH_NODE): REAL_64
			-- Traversal cost from node b to neighbouring node c
		do
			Result := tm.euclidean_distance (b.position, c.position)
		end

	tot_expected_cost (a, b, c, d: separate SPATIAL_GRAPH_NODE): REAL_64
			-- Total expected cost
		do
			Result := path_cost (a, b, c) + cost (c, d)
		end

feature {NONE}

	tm: TRIGONOMETRY_MATH

end -- class
