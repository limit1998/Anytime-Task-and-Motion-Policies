(define (domain hanoi)
  (:requirements :strips)
  (:types
    object
  )
  (:predicates
    (clear ?x - object)
    (on ?x - object ?y - object)
    (bigger ?x - object ?y - object)
    (handempty)
    (inGripper ?p - object)
  )

  (:action pick
    :parameters (?box ?from)
    :precondition (and (handempty)
                       (on ?box ?from)
                       (clear ?box)
                  )
    :effect (and (inGripper ?box)
                 (clear ?from)
                 (not (handempty))
                 (not (on ?box ?from))
            )
  )

  (:action place
    :parameters (?box ?to)
    :precondition (and (not (handempty))
                       (not (bigger ?box ?to))
                       (clear ?to)
                       (inGripper ?box)
                  )
    :effect (and
                 (on ?box ?to)
                 (not (clear ?to))
                 (handempty)
                 (not (inGripper ?box))
            )
  )
)