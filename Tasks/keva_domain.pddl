(define (domain keva)
  (:requirements :strips :typing)
  (:types plank region orientated group)
  (:predicates
  		   (onTable ?p - plank)
  		   (on ?r - region ?r - region)
  		   (clearRegion ?r - region)
         (clearPlank ?p - plank)
  		   (handempty)
  		   (inGripper ?p - plank)
  		   (clearTable)
  		   (belongTo ?r - region ?p - plank)
  		   (orientation ?p - plank ?o - orientated)
         (sideGroup ?r))
  (:constants 
         vertical - orientated horizontal - orientated)

  (:action pickUp_plank_from_region
  		 :parameters (?rob - robot ?p - plank ?g - group)
  		 :precondition (and (handempty) 
                          (clearPlank ?p))
  		 :effect (and (not (handempty)) 
                    (inGripper ?p)
                    (not (clearPlank ?p)))
  )

  (:action putDown_plank_vertically_onTable
  		 :parameters (?rob - robot ?p - plank ?r - region)
  		 :precondition (and (not (handempty))
                          (inGripper ?p)
                          (sideGroup ?r)
                          (belongTo ?r ?p))
  		 :effect (and (handempty)
                    (onTable ?p)
                    (not (clearRegion ?r))
                    (orientation ?p vertical)
                    (not (inGripper ?p)))
  )

  (:action putDown_plank_horizontally_onTable
  		 :parameters (?rob - robot ?p - plank)
  		 :precondition (and (not (handempty))
                          (inGripper ?p))
  		 :effect (and (handempty)
                    (onTable ?p)
                    (orientation ?p horizontal)
                    (not (inGripper ?p)))
  )

  (:action putDown_plank_vertically_onPlank
  		 :parameters (?rob - robot ?p1 - plank ?p2 - plank ?r1 - region ?r2 - region)
  		 :precondition (and (not (handempty))
                          (inGripper ?p1)
                          (clearRegion ?r2)
                          (belongTo ?r1 ?p1)
                          (belongTo ?r2 ?p2) 
                          (sideGroup ?r1)
                          (not (= ?p1 ?p2)))
  		 :effect (and (handempty)
                    (not (clearRegion ?r2))
                    (on ?r1 ?r2)
                    (not (clearPlank ?p2))
                    (orientation ?p1 vertical)
                    (not (inGripper ?p1))
                    (not (clearPlank ?p2)))
  )

  (:action putDown_plank_horizontally_onSinglePlank
  		 :parameters (?rob - robot ?p1 - plank ?p2 - plank ?r1 - region ?r2 - region)
  		 :precondition (and (not (handempty))
                          (inGripper ?p1)
                          (clearRegion ?r2)
                          (belongTo ?r1 ?p1)
                          (belongTo ?r2 ?p2)
                          (not (= ?p1 ?p2)))
  		 :effect (and (handempty)
                    (not (inGripper ?p1))
                    (on ?r1 ?r2)
                    (not (clearPlank ?p2))
                    (orientation ?p1 horizontal)
                    (not (clearRegion ?r1))
                    (not (clearRegion ?r2))
                    (not (clearPlank ?p2)))
  )

  (:action putDown_plank_horizontally_onDoublePlank
  		 :parameters (?rob - robot ?p1 - plank ?p2 - plank ?p3 - plank ?r1 - region ?r2 - region ?r3 - region ?r4 - region)
  		 :precondition (and (not (handempty))
                          (inGripper ?p1)
                          (clearRegion ?r3)
                          (clearRegion ?r4)
                          (belongTo ?r1 ?p1)
                          (belongTo ?r2 ?p1)
                          (belongTo ?r3 ?p2)
                          (belongTo ?r4 ?p3)
                          (sideGroup ?r3)
                          (sideGroup ?r4)
                          ;(orientation ?p2 vertical)
                          ;(orientation ?p3 vertical)
                          (not (= ?p1 ?p2))
                          (not (= ?p1 ?p3))
                          (not (= ?p2 ?p3)))
  		 :effect (and (handempty)
                    (not (inGripper ?p1))
                    (on ?r1 ?r3)
                    (on ?r2 ?r4)
                    (not (clearRegion ?r3))
                    (not (clearRegion ?r4))
                    (orientation ?p1 horizontal)
                    (not (clearPlank ?p2))
                    (not (clearPlank ?p3)))
  )
)