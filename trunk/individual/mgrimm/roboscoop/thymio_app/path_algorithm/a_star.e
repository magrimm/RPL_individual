note
	description: "Summary description for {A_STAR}."
	author: "Marius Grimm"
	date: "30.10.15"

class
	A_STAR

feature

	a_star_algorithm (start_node, goal_node: POINT_MSG): PATH_MSG
		local
--			g: GRID_GRAPH
			start_pos, goal_pos: SPATIAL_GRAPH_NODE
		do
--			start_pos := g.node_at (start_node.x.truncated_to_integer,
--									start_node.y.truncated_to_integer,
--									1)
--			goal_pos := g.node_at (goal_node.x.truncated_to_integer,
--									goal_node.y.truncated_to_integer,
--									1)
			Result := create {PATH_MSG}.make_empty -- only for testing
		end

end -- class
