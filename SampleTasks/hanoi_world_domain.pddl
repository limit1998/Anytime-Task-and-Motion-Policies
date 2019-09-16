(define (domain hanoi)
  (:requirements :strips)
  (:types
    object
  )
  (:predicates
    (clear ?x - object)
    (on ?x - object ?y - object)
    (bigger ?x - object ?y - object)
  )

  (:action move
    :parameters (?box ?from ?to)
    :precondition (and
                    (not (bigger ?box ?to))
                    (on ?box ?from)
                    (clear ?box)
                    (clear ?to)
    )
    :effect (and
                (clear ?from)
                (on ?box ?to)
                (not (on ?box ?from))
		(not (clear ?to))
    )
  )
)
