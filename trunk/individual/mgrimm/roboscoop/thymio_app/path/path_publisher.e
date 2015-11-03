note
	description: "Publisher class for publishing PATH_MSG."
	author: "Marius Grimm"
	date: "29.10.15"

class
	PATH_PUBLISHER

create
	make_with_topic

feature {NONE}

	make_with_topic (topic_name: separate STRING)
			-- Create Current.
		do
			create publisher.make_with_topic (topic_name)
			publisher.advertize (12, True)
		end

feature -- access

	set_path_with_point_msg (a_val_1, a_val_2: POINT_MSG; header_topic: STRING)
			-- Publish path from two POINT_MSG
		local
			a_header: HEADER_MSG

			a_orientation1, a_orientation2: QUATERNION_MSG
			a_position1, a_position2: POINT_MSG
			a_pose1, a_pose2: POSE_MSG
			a_default_value1, a_default_value2: POSE_STAMPED_MSG

			a_poses: ARRAY [POSE_STAMPED_MSG]
			path: PATH_MSG

		do
			-- Header
			a_header := create {HEADER_MSG}.make_now (header_topic)

			-- First point
			a_orientation1 := create {QUATERNION_MSG}.make_empty
			a_position1 := create {POINT_MSG}.make_with_values (a_val_1.x, a_val_1.y, a_val_1.z)
			a_pose1 := create {POSE_MSG}.make_with_values (a_position1, a_orientation1)
			a_default_value1 := create {POSE_STAMPED_MSG}.make_with_values (a_header, a_pose1)

			-- Second point
			a_orientation2 := create {QUATERNION_MSG}.make_empty
			a_position2 := create {POINT_MSG}.make_with_values (a_val_2.x, a_val_2.y, a_val_2.z)
			a_pose2 := create {POSE_MSG}.make_with_values (a_position2, a_orientation2)
			a_default_value2 := create {POSE_STAMPED_MSG}.make_with_values (a_header, a_pose2)

			-- Save points in array
			a_poses := create {ARRAY [POSE_STAMPED_MSG]}.make_filled (a_default_value1, 1, 2)
			a_poses[2] := a_default_value2

			-- Publish path from saved points in array a_poses
			path := create {PATH_MSG}.make_with_values (a_header, a_poses)
			publisher.publish (path)
		end

	set_path_with_path_msg (path: PATH_MSG)
		-- Publish path from PATH_MSG
		do
			publisher.publish (path)
		end

	set_path_with_spatial_graph_nodes (sgn: ARRAYED_LIST[SPATIAL_GRAPH_NODE]; header_topic: STRING)
		-- Publish path from spatial_graph_nodes
		local
			i: INTEGER
			a_header: HEADER_MSG

			a_orientation: QUATERNION_MSG
			a_position: POINT_MSG
			a_pose: POSE_MSG
			a_default_value: POSE_STAMPED_MSG

			a_poses: ARRAY [POSE_STAMPED_MSG]
			path: PATH_MSG

		do
			-- Header
			create a_header.make_now (header_topic)
			-- Empty array of POSE_STAMPED_MSG
			create a_poses.make_empty

			from
				i := 1
			until
				i > sgn.count
			loop
				-- Create point
				create a_orientation.make_empty
				create a_position.make_with_values (sgn.at (i).position.x, sgn.at (i).position.y, sgn.at (i).position.z)
				create a_pose.make_with_values (a_position, a_orientation)
				create a_default_value.make_with_values (a_header, a_pose)

				-- Save points in array
				a_poses.force (a_default_value, i)

				i := i + 1
			end

			-- Publish path from saved points in array a_poses
			create path.make_with_values (a_header, a_poses)
			publisher.publish (path)
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [PATH_MSG]
			-- Publisher object.

end -- class
