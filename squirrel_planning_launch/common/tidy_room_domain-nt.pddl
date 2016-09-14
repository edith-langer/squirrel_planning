(define (domain squirrel_tidy_room)

(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions)

(:types
	robot
	area
	waypoint
)

(:predicates
	(explored ?a - area)
	(examined ?a - area)
	(robot_in ?v - robot ?a - area)
	(robot_at ?v - robot ?wp - waypoint)
	(connected ?from ?to - area)
	(tidy ?a - area)
	(accessible ?from ?to - area)
)

(:constants
dummy - waypoint
)

(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and
		(robot_at ?v ?from))
	:effect (and
		(not (robot_at ?v ?from))
		(robot_at ?v ?to)
	)
)

;;(:action goto_waypoint
;;	:parameters (?v - robot ?from ?to - area)
;;	:precondition (and
;;		(robot_in ?v ?from)
;;		(accessible ?from ?to)
;;	)
;;	:effect (and
;;		(not (robot_in ?v ?from))
;;		(robot_in ?v ?to)
;;	)
;;)

(:action clear_connection
	:parameters (?v - robot ?from ?to - area)
	:precondition (and
		(robot_in ?v ?from)
		(connected ?from ?to)
	)
	:effect (and
		(accessible ?from ?to)
	)
)

(:action explore_area
	:parameters (?v - robot ?a - area)
	:precondition (and
		(robot_in ?v ?a)
	)
	:effect (and
		(explored ?a)
	)
)

(:action examine_area
	:parameters (?v - robot ?a - area)
	:precondition (and
		(robot_in ?v ?a)
		(explored ?a)
	)
	:effect (and
		(examined ?a)
	)
)

(:action tidy_area
	:parameters (?v - robot ?a - area)
	:precondition (and
		(robot_in ?v ?a)
		(examined ?a)
	)
	:effect (and
		(tidy ?a)
	)
)

)

