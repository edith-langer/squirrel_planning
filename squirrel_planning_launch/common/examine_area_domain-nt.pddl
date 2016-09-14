(define (domain squirrel_explore)

(:requirements :strips :typing  :disjunctive-preconditions :negative-preconditions)

(:types
	waypoint
	robot
	object
)

(:predicates
	(examined ?o - object)
;;	(explored ?wp - waypoint)
	(robot_at ?v - robot ?wp - waypoint)
	(observable_from ?o - object ?wp- waypoint)
)

;; Use perception actions to search for objects at the current waypoint
(:action examine_object
	:parameters (?v - robot ?wp - waypoint ?o - object)
	:precondition (and
		(robot_at ?v ?wp)
		(observable_from ?o ?wp)
	)
	:effect (and
		(examined ?wp)
	)
)

;; Move between any two waypoints, avoiding terrain
(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and
		(robot_at ?v ?from))
	:effect (and
		(not (robot_at ?v ?from))
		(robot_at ?v ?to)
	)
)
)

