(define (domain kitchen)
  (:requirements :strips :typing :adl )

  (:types
    tray
    item
    location
    robot
    tray
  )

  (:predicates

    (on_tray ?item - item ?tray - tray)
    (one)
    (two)
    (three)
    (at ?item - item ?loc - location)
    (broken ?item)
    (near_loc ?rob - robot ?loc - location)
    (init)
    (terminated)
    (handempty ?rob - robot)
    (holding ?rob - robot ?item - item)
    (first)
    (second)
    (third)
    (available)
    (m1)
    (m2)
    (m3)
    (m4)
    (at_tray ?tray - tray ?loc - location)
    (holding_tray ?rob - robot ?tray - tray)
    )


  ; (:action init 
  ;   :parameters()
  ;   :precondition(and 
  ;     (not (init))
  ;   )
  ;   :effect(and 
  ;     (init)
  ;   ;facts here 
  ;   )
  ; )

  (:action pick_from_location 
    :parameters (?item - item ?rob - robot ?loc - location)
    :precondition (and 
      (at ?item ?loc)
      (handempty ?rob)
      (near_loc ?rob ?loc)
      (available)
      (init)
    )
    :effect(and 
      (holding ?rob ?item)
      (not (at ?item ?loc))
      (not (handempty ?rob))
    )
  )


  (:action pick_tray_from_location
    :parameters (?tray - tray ?loc - location ?rob - robot)
    :precondition (and
        (at_tray ?tray ?loc)
        (handempty ?rob)
        (near_loc ?rob ?loc)
         (available)
        (init)
        (not (three))
    )
    :effect (and
        (holding_tray ?rob ?tray)
        (not (at_tray ?tray ?loc))
        (not (handempty ?rob))
    )
  )

  (:action pick_tray_from_location_2
    :parameters (?tray - tray ?loc - location ?rob - robot)
    :precondition (and
        (at_tray ?tray ?loc)
        (handempty ?rob)
        (near_loc ?rob ?loc)
         (available)
        (init)
        (three)
    )
    :effect (and
        (holding_tray ?rob ?tray)
        (not (at_tray ?tray ?loc))
        (not (handempty ?rob))

        (probabilistic
            0.7 (and (forall (?i - item)
                        (and
                            (not (on_tray ?i ?tray))
                            (broken ?i)
                          )
                    
                ) (not (three)))
        )
    )
  )


  (:action pick_from_tray
  
      :parameters (?item - item ?rob - robot ?tray - tray ?loc - location)
      :precondition (and 
        (on_tray ?item ?tray)
        (at_tray ?tray ?loc)
        (handempty ?rob)
        (near_loc ?rob ?loc)
        (available)
        (init)
      )
      :effect (and  
        (not (on_tray ?item ?tray))
        (not (handempty ?rob))
        (holding ?rob ?item)
        (when (one)
            (not (one))
        )
        (when (two)
            (and (not (two)) (one))
        )
        (when (three)
            (and (not (three))(two))
        )
      )
  )


  (:action place_on_tray
    :parameters (?item - item ?rob - robot ?tray - tray ?loc - location)
    :precondition (and 
      (at_tray ?tray ?loc)
      (holding ?rob ?item)
      (near_loc ?rob ?loc)
      (available)
      (init)
      (not (three))
    )
    :effect(and
      (not (holding ?rob ?item))
      (on_tray ?item ?tray)
      (handempty ?rob)
       (when (two)
           (and (not (two))(three))
       )
       (when (one)
           (and (not (one))(two))
       )
       (when (and (not (one))(not (two)) )
            (one)
        )
    )
  )

  (:action place_at_location
    :parameters (?item - item ?rob - robot ?loc - location)
    :precondition (and 
      (holding ?rob ?item)
      (near_loc ?rob ?loc)
      (available)
      (init)
    )
    :effect (and 
      (not (holding ?rob ?item))
      (handempty ?rob)
      (at ?item ?loc)
    )
  )

  (:action place_at_location_tray
    :parameters (?tray - tray ?rob - robot ?loc - location)
    :precondition (and
        (holding_tray ?rob ?tray)
        (near_loc ?rob ?loc)
        (available)
        (init)
    )
    :effect (and
        (not (holding_tray ?rob ?tray))
        (handempty ?rob)
        (at_tray ?tray ?loc)
    )

  )

    (:action c1
        :parameters ()
        :precondition (and
            (not (available))
        )
        :effect(and
            (first)
        )
    )

    (:action c2
        :parameters ()
        :precondition (and
            (not (available))
            (first)
        )
        :effect(and
            (second)
            (not (first))
        )
    )

    (:action c3
        :parameters ()
        :precondition (and
            (not (available))
            (second)
        )
        :effect(and
            (not (second))
            (third)
        )
    )

    (:action c4
        :parameters ()
        :precondition (and
            (not (available))
            (third)
        )
        :effect(and
            (not (third))
            (available)
        )
    )



  (:action move
      :parameters (?rob - robot ?src - location ?dest - location)
      :precondition (and 
        (near_loc ?rob ?src)
        (init)
        (available)
      )
      :effect (and 
        (not (near_loc ?rob ?src))
        (near_loc ?rob ?dest)
        (not (available))
        (when (m3) (m4))
        (when (m2) (m3))
        (when (m1) (m2))
        (m1)
      )
  )

  (:action done 
    :parameters ()
    :precondition (and
        (available)
      (not (terminated))
          (forall (?i - item) (and (at ?i table_2) (not (broken ?i ))))
          (not (m4))
    )
    :effect (and 
      (terminated)
    )
  
  )
)




(define (problem p01) 
  
  (:domain kitchen)
  (:objects 
    fetch - robot
    table_1 table_2 - location
    plate_1 cup_1 plate_2 cup_2 - item
    t - tray
  )

  (:init
      (at plate_1 table_1)
      (at plate_2 table_1)
      (at cup_2 table_1)
      (at cup_1 table_1)
      (handempty fetch)
      (near_loc fetch table_1)
       (at_tray t table_1)
      (init)
      (available)
  )

  (:goal (and
      (terminated)
      (forall (?i - item) (and (at ?i table_2) (not (broken ?i ))))
      (not (m4))

    
  ))

)