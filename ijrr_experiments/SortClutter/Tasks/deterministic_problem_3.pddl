(define (problem p01)

	(:domain sort_clutter)
	(:objects

		gripper - manip
		can1 can2 can3 can4 can5 can6 can7 can8 can9 can10 can11 can12  - can
		traj - trajectory
		fetch - robot
		main_table blue_table green_table - table
		blue green red - color

	)

	(:init
        (empty gripper)
        (at_obj can1 main_table)
(at_obj can2 main_table)
(at_obj can3 main_table)
(at_obj can4 main_table)
(at_obj can5 main_table)
(at_obj can6 main_table)
(at_obj can7 main_table)
(at_obj can8 main_table)
(at_obj can9 main_table)
(at_obj can10 main_table)
(at_obj can11 main_table)
(at_obj can12 main_table)

        (color can1 green)
(color can2 green)
(color can3 green)
(color can4 blue)
(color can5 blue)
(color can6 blue)
(color can7 red)
(color can8 red)
(color can9 red)
(color can10 red)
(color can11 red)
(color can12 red)

	)

	(:goal (and
	    (forall (?can - can)(or (not (color ?can green))(at_obj ?can green_table)))
	    (forall (?can - can)(or (not (color ?can blue))(at_obj ?can blue_table)))
	    (forall (?can - can)(or (not (color ?can red))(at_obj ?can main_table)))
	))

)