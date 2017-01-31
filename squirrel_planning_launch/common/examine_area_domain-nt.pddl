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
	(observable_from ?o - object ?wp - waypoint)
	(camera_aimed ?v - robot)
	(camera_neutral ?v - robot)
)

;; Use perception actions to search for objects at the current waypoint
(:action examine_object
	:parameters (?v - robot ?wp - waypoint ?o - object)
	:precondition (and
		(robot_at ?v ?wp)
		(observable_from ?o ?wp)
		(camera_aimed ?v)
	)
	:effect (and
		(examined ?o)
	)
)

;; Move between any two waypoints, avoiding terrain
(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and
		(robot_at ?v ?from)
		(camera_neutral ?v)
	)
	:effect (and
		(not (robot_at ?v ?from))
		(robot_at ?v ?to)
	)
)

(:action aim_camera
	:parameters (?v - robot)
	:precondition (camera_neutral ?v)
	:effect (and
		(not (camera_neutral ?v))
		(camera_aimed ?v)
	)
)

(:action reset_camera
	:parameters (?v - robot)
	:precondition (camera_aimed ?v)
	:effect (and
		(not (camera_aimed ?v))
		(camera_neutral ?v)
	)
)
)

