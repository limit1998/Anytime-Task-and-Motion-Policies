(define (problem p01)

	(:domain sort_clutter)
	(:objects

		gripper - manip
		can1 can2 can3 can4 can5 can6 can7 can8 can9 can10 can11 can12 can13 can14 can15 can16 can17 can18 can19 can20  - can
		traj - trajectory
		fetch - robot
		main_table blue_table green_table - table
		blue green - color

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
(at_obj can13 main_table)
(at_obj can14 main_table)
(at_obj can15 main_table)
(at_obj can16 main_table)
(at_obj can17 main_table)
(at_obj can18 main_table)
(at_obj can19 main_table)
(at_obj can20 main_table)

        (color can1 green)
(color can2 green)
(color can3 green)
(color can4 green)
(color can5 green)
(color can6 green)
(color can7 green)
(color can8 green)
(color can9 green)
(color can10 green)
(color can11 blue)
(color can12 blue)
(color can13 blue)
(color can14 blue)
(color can15 blue)
(color can16 blue)
(color can17 blue)
(color can18 blue)
(color can19 blue)
(color can20 blue)

	)

	(:goal (and
	    (forall (?can - can)(or (not (color ?can green))(at_obj ?can green_table)))
	    (forall (?can - can)(or (not (color ?can blue))(at_obj ?can blue_table)))
	))

)