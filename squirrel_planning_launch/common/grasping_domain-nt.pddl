(define (domain squirrel_tidy_room)

(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions)

(:types
	robot
	waypoint
	object
)

(:predicates
	(robot_at ?r - robot ?wp - waypoint)
	(grasped ?o - object)
)

(:action pickup_object
	:parameters (?r - robot ?wp - waypoint ?ob - object)
	:precondition (and
		(robot_at ?r ?wp)
	)
	:effect (and 
		(grasped ?o)
	)
)

(:action putdown_object
	:parameters (?r - robot ?wp - waypoint ?ob - object)
	:precondition (and
		(robot_at ?r ?wp)
	)
	:effect (and 
		(not (grasped ?o))
	)
)
)

