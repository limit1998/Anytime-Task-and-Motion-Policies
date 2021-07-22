(define (problem p01)

	(:domain sort_clutter)
	(:objects

		gripper - manip
		##OBJECTS_HERE## - can
		traj - trajectory
		fetch - robot
		main_table blue_table green_table - table
		blue green red - color

	)

	(:init
        (empty gripper)
        ##OBJ_INIT_LOC_HERE##
        ##COLORS_HERE##
	)

	(:goal (and
	    (forall (?can - can)(or (not (color ?can green))(at_obj ?can green_table)))
	    (forall (?can - can)(or (not (color ?can blue))(at_obj ?can blue_table)))
	    (forall (?can - can)(or (not (color ?can red))(at_obj ?can main_table)))
	))

)