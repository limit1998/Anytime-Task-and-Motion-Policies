(define (domain manuf_world_small)


(:requirements :strips :adl :typing :negative-preconditions :equality)


(:types
    robot
    location
    process
    machine
    object
    gripper
)


(:predicates 
    (robot_at ?bot - robot ?loc - location)
    (empty ?g - gripper)
    (at ?obj - object ?loc - location)
    (machine_at ?machine - machine ?loc - location)
    (can_perform ?machine - machine ?process - process)
    (process_completed ?p - process)
    (holding ?bot - robot ?obj - object)
)

(:action pick
    :parameters (?bot - robot ?loc - location ?obj - object ?grippr - gripper)
    :precondition (and 
            (robot_at ?bot ?loc)
            (empty ?grippr)
            (at ?obj ?loc)
            (not (holding ?bot ?obj))
        )
    :effect (and
        (not (at ?obj ?loc))
        (not (empty ?grippr))
        (holding ?bot ?obj)
    )
)

(:action place
    :parameters (?obj - object ?bot - robot ?loc - location ?machine - machine ?grippr - gripper)
    :precondition (and
        (robot_at ?bot ?loc)
        (not (empty ?grippr))
        (machine_at ?machine ?loc)
        (holding ?bot ?obj)
    )
    :effect (and
        (empty ?grippr)
        (at ?obj ?loc)
        (not (holding ?bot ?obj))
    )
)

(:action run_machine
    :parameters (?m - machine ?bot - robot ?loc - location ?obj - object ?p - process)
    :precondition (and 
        (robot_at ?bot ?loc)
        (machine_at ?m ?loc)
        (at ?obj ?loc)
        (can_perform ?m ?p)

        (or (and (= ?p process_1) (process_completed init_process))
            (and (= ?p process_2) (process_completed process_1))
            (and (= ?p process_3) (process_completed process_2))
            (and (= ?p process_4) (process_completed process_3))
        )
    )
    :effect (and 
        (process_completed ?p)
    )
)


(:action move
    :parameters (?bot - robot ?oldloc - location ?loc - location)
    :precondition (and
        (robot_at ?bot ?oldloc)
    )
    :effect (and
        (robot_at ?bot ?loc)
        (not (robot_at ?bot ?oldloc))
    )
)

)