(define (domain sort_clutter)
    (:requirements :typing :strips :adl :equality)
    (:types
        can
        manip
        pose
        trajectory
        robot
        table
        color
    )
    (:predicates
        (obstructs ?traj - trajectory ?obstructingobj ?obstructedobj - can ?table - table)
        (empty ?gripper - manip)
        (ingripper ?obj - can ?gripper - manip)
        (at_obj ?obj - can ?tbl - table)
        (color ?obj - can ?clr - color)
        (putdownobstructs ?traj - trajectory ?o ?obj - can ?table - table)
    )
    (:action grasp
        :parameters(?obj - can ?r - robot ?tbl - table ?g - manip ?traj - trajectory)
        :precondition(and
            (empty ?g)
            (at_obj ?obj ?tbl)
            (forall (?o - can)(not (obstructs ?traj ?o ?obj ?tbl)))
        )
        :effect(and
            (not (empty ?g))
            (ingripper ?obj ?g)
            (not (at_obj ?obj ?tbl))
            (forall (?o - can) (forall (?t - trajectory) (forall (?tb - table) (not (obstructs ?t ?obj ?o ?tb)))))
            (forall (?o - can) (forall (?t - trajectory) (forall (?tb - table) (not (putdownobstructs ?t ?obj ?o ?tb)))))
        )
    )
    (:action put
        :parameters(?obj - can ?r - robot ?tbl - table ?g - manip ?traj - trajectory)
        :precondition(and
            (ingripper ?obj ?g)
            (forall (?o -can)(not (putdownobstructs ?traj ?o ?obj ?tbl)))
        )
        :effect(and
            (not (ingripper ?obj ?g))
            (empty ?g)
            (at_obj ?obj ?tbl)
        )
    )
)