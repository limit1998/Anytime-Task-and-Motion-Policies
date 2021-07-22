(define (problem p01)

	(:domain sort_clutter)
	(:objects

		gripper - manip
		can1 can2  - can
		traj - trajectory
		fetch - robot
		main_table blue_table green_table - table
		blue green - color

	)

	(:init
        (empty gripper)
        (at_obj can1 main_table)
(at_obj can2 main_table)

        (color can1 green)
(color can2 blue)

	)

	(:goal (and
	    (forall (?can - can)(or (not (color ?can green))(at_obj ?can green_table)))
	    (forall (?can - can)(or (not (color ?can blue))(at_obj ?can blue_table)))
	))

)