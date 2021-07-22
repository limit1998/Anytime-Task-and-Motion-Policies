(define (problem p01)

(:domain manuf_world_small)
(:objects
    fetch - robot
    fetch_init_loc - location
    machine_1 machine_2 machine_3 machine_4 machine_5 machine_6 machine_7 machine_8 machine_9 machine_10 machine_11 machine_12 machine_13 machine_14 machine_15 machine_16 - machine
    machine_1_loc machine_2_loc machine_3_loc machine_4_loc machine_5_loc machine_6_loc machine_7_loc machine_8_loc machine_9_loc machine_10_loc machine_11_loc machine_12_loc machine_13_loc machine_14_loc machine_15_loc machine_16_loc - location
    init_process process_1 process_2 process_3 process_4 - process
    obj - object
    obj_location - location
    grippr - gripper
)

(:init 
    
    (robot_at fetch fetch_init_loc)
    (at obj obj_location)
    (empty grippr)
    ; (holding fetch obj)
    
    (machine_at machine_1 machine_1_loc)
    (machine_at machine_2 machine_2_loc)
    (machine_at machine_3 machine_3_loc)
    (machine_at machine_4 machine_4_loc)
    (machine_at machine_5 machine_5_loc)
    (machine_at machine_6 machine_6_loc)
    (machine_at machine_7 machine_7_loc)
    (machine_at machine_8 machine_8_loc)
    (machine_at machine_9 machine_9_loc)
    (machine_at machine_10 machine_10_loc)
    (machine_at machine_11 machine_11_loc)
    (machine_at machine_12 machine_12_loc)
    (machine_at machine_13 machine_13_loc)
    (machine_at machine_14 machine_14_loc)
    (machine_at machine_15 machine_15_loc)
    (machine_at machine_16 machine_16_loc)
    
    (can_perform machine_1 process_1)
    (can_perform machine_2 process_1)
    (can_perform machine_3 process_1)
    (can_perform machine_4 process_1)
    (can_perform machine_5 process_2)
    (can_perform machine_6 process_2)
    (can_perform machine_7 process_2)
    (can_perform machine_8 process_2)
    (can_perform machine_9 process_3)
    (can_perform machine_10 process_3)
    (can_perform machine_11 process_3)
    (can_perform machine_12 process_3)
    (can_perform machine_13 process_4)
    (can_perform machine_14 process_4)
    (can_perform machine_15 process_4)
    (can_perform machine_16 process_4)
    
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