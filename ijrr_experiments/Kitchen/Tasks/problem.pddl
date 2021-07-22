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