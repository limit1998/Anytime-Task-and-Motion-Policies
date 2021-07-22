(define (problem p01)

(:domain manuf_world_small)
(:objects
    fetch - robot
    fetch_init_loc - location
    machine2 machine11 machine14 machine4 - machine
    machine2_loc machine11_loc machine14_loc machine4_loc - location
    init_process process_1 process_2 process_3 process_4 - process
    ply_object - object
    object_init_location - location
    grippr - gripper
)

(:init 
    
    (robot_at fetch fetch_init_loc)
    (at ply_object object_init_location)
    (empty grippr)
    ; (holding fetch obj)
    
    (machine_at machine2 machine2_loc)
    (machine_at machine11 machine11_loc)
    (machine_at machine14 machine14_loc)
    (machine_at machine4 machine4_loc)
    
    (can_perform machine2 process_1)
    (can_perform machine11 process_2)
    (can_perform machine14 process_3)
    (can_perform machine4 process_4)
    
    (process_completed init_process)

    ; (= (previous_process process_1) init_process)
    
)

(:goal (and
    (process_completed process_1)
    (process_completed process_2)
    (process_completed process_3)
    (process_completed process_4)
))
)