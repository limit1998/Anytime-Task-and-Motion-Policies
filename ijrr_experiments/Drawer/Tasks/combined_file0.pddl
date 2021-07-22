(define (domain drawers)

    (:requirements :strips :typing :adl :mdp)

    (:types
        robot
        can
        drawer
    )

    (:predicates
        (in ?c - can ?d - drawer)
        (holding ?c - can)
        (hand_empty ?r - robot)
        (closed ?d - drawer)
        (opened ?d - drawer)
        (obstructs ?d - drawer ?c - can)
        (obstructs_dd ?d1 ?d2 - drawer)
        (init)
        (dummy)
        (object_spawned ?c - can)
        (terminated)
        (visible ?c - can)
    )

    (:action pick
        :parameters (?c - can ?r - robot ?d - drawer)
        :precondition(and
            (init)
            (in ?c ?d)
            (hand_empty ?r)
            (forall (?d1 - drawer)(not (obstructs ?d1 ?c)))
            (visible ?c)
        )
        :effect(and
            (holding ?c)
            (not (in ?c ?d))
            (not (hand_empty ?r))
        )
    )

    (:action open
        :parameters (?d - drawer ?r - robot)
        :precondition(and
            (init)
            (hand_empty ?r)
            (forall (?d1 - drawer)(or (=?d1 ?d)(not (obstructs_dd ?d1 ?d))))
            (closed ?d)
        )
        :effect(and
            (forall (?c - can)(not (obstructs ?d ?c)))
            (opened ?d)
            (not (closed ?d))
            (forall (?c - can)
                (when (in ?c ?d)
                    (visible ?c)
                )
            )
        )

    )

    (:action spawn_can
        :parameters (?c - can ?r - robot )
        :precondition (and
            (not (object_spawned ?c))
        )
        :effect(and
            (probabilistic
                0.6 (in ?c top_drawer)
                0.4 (in ?c bottom_drawer)
            )
            (object_spawned ?c)
            (init)
            (closed top_drawer)
            (closed bottom_drawer)
            (hand_empty ?r)
        )
    )

    (:action close
        :parameters (?d - drawer ?r - robot)
        :precondition(and
            (init)
            (hand_empty ?r)
            (opened ?d)
            (forall (?d1 - drawer)(or (=?d1 ?d)(not (obstructs_dd ?d1 ?d))))
        )
        :effect(and
            (closed ?d)
            (not (opened ?d))
            (forall (?d1 - drawer)
                (when (not (= ?d1 ?d))
                    (not (obstructs_dd ?d1 ?d))
                )
            )
            (forall (?c - can)
                (when (in ?c ?d)
                    (not (visible ?c))
                )
            )
        )

    )

    (:action done
        :parameters ()
        :precondition (and
            (holding can1)
            (not (terminated))
        )
        :effect(and
            (terminated)
        )
    )
)
(define (problem p01)
    (:domain drawers)

    (:objects
        top_drawer bottom_drawer - drawer
        can1 - can
        fetch - robot
    )

    (:init
        (dummy)
    )

    (:goal
        (and
            (holding can1)
            (terminated)
        )
    )

)