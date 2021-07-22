(define (problem p01)

	(:domain sort_clutter)
	(:objects

		gripper - manip
		can1 can2 can3 can4 can5 can6 can7 can8 can9 can10 can11 can12 can13 can14 can15 can16 can17 can18 can19 can20  - can
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
(color can6 blue)
(color can7 blue)
(color can8 blue)
(color can9 blue)
(color can10 blue)
(color can11 red)
(color can12 red)
(color can13 red)
(color can14 red)
(color can15 red)
(color can16 red)
(color can17 red)
(color can18 red)
(color can19 red)
(color can20 red)

	)

	(:goal (and
	    (forall (?can - can)(or (not (color ?can green))(at_obj ?can green_table)))
	    (forall (?can - can)(or (not (color ?can blue))(at_obj ?can blue_table)))
	    (forall (?can - can)(or (not (color ?can red))(at_obj ?can main_table)))
	))

)