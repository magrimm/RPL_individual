note
	description: "Summary description for {PATH_PUBLISHER}."
	author: ""
	date: "$Date$"

class
	PATH_PUBLISHER

create
	make_with_topic

feature

	make_with_topic (topic_name: separate STRING)
			-- Create Current.
		do
			create publisher.make_with_topic (topic_name)
			publisher.advertize (12, True)
		end

feature

	set_path (a_x1, a_y1, a_z1, a_x2, a_y2, a_z2: REAL_64)
			-- Publishing speed.
		local
			a_orientation1: QUATERNION_MSG
			a_position1: POINT_MSG
			a_pose1: POSE_MSG
			a_default_value1: POSE_STAMPED_MSG


			a_orientation2:QUATERNION_MSG
			a_position2: POINT_MSG
			a_pose2: POSE_MSG
			a_default_value2: POSE_STAMPED_MSG

			a_header: HEADER_MSG
			a_poses: ARRAY [POSE_STAMPED_MSG]
			path: PATH_MSG

		do
			a_header := create {HEADER_MSG}.make_now ("/path")

			a_orientation1 := create {QUATERNION_MSG}.make_with_values (0, 0, 0, 0)
			a_position1 := create {POINT_MSG}.make_with_values (a_x1, a_y1, a_z1)
			a_pose1 := create {POSE_MSG}.make_with_values (a_position1, a_orientation1)
			a_default_value1 := create {POSE_STAMPED_MSG}.make_with_values (a_header, a_pose1)


			a_orientation2 := create {QUATERNION_MSG}.make_with_values (0, 0, 0, 0)
			a_position2 := create {POINT_MSG}.make_with_values (a_x2, a_y2, a_z2)
			a_pose2 := create {POSE_MSG}.make_with_values (a_position2, a_orientation2)
			a_default_value2 := create {POSE_STAMPED_MSG}.make_with_values (a_header, a_pose2)

			a_poses := create {ARRAY [POSE_STAMPED_MSG]}.make_filled (a_default_value1, 1, 2)
			a_poses[2] := a_default_value2

			path := create {PATH_MSG}.make_with_values (a_header, a_poses)
			publisher.publish (path)
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [PATH_MSG]
			-- Publisher object.

end
