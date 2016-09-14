(define (domain find_key)
(:requirements :typing :conditional-effects :negative-preconditions :disjunctive-preconditions)

(:types
	waypoint robot object box type
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(object_at ?o - object ?wp - waypoint)
	(box_at ?b - box ?wp - waypoint)
	(gripper_empty ?v - robot)
	(holding ?v - robot ?o - object)
	(tidy ?o - object)
	(push_location ?o - object ?wp - waypoint)
	(can_pickup ?v - robot ?t - type)
	(can_push ?v - robot ?t - type)
	(can_fit_inside ?t - type ?b - box)
	(inside ?o - object ?b - box)
	(near_for_grasping ?wp1 ?wp2 - waypoint)
	(near_for_pushing ?wp1 ?wp2 - waypoint)
	(is_of_type ?o - object ?t -type)
)

(:action put_object_in_box
	:parameters (?v - robot ?wp ?near_wp - waypoint ?o1 - object ?b - box ?t - type)
	:precondition (and
		(box_at ?b ?wp)
		(robot_at ?v ?near_wp)
		(near_for_grasping ?near_wp ?wp)
		(holding ?v ?o1)
		(can_fit_inside ?t ?b)
		(is_of_type ?o1 ?t)
	)
	:effect (and
		(and
			(not (holding ?v ?o1))
			(gripper_empty ?v)
			(inside ?o1 ?b)
		)
	)
)

(:action pickup_object
	:parameters (?v - robot ?wp ?near_wp - waypoint ?o - object ?t - type)
	:precondition (and
		(robot_at ?v ?near_wp)
		(object_at ?o ?wp)
		(gripper_empty ?v)
		(can_pickup ?v ?t)
		(is_of_type ?o ?t)
		(near_for_grasping ?near_wp ?wp)
	)
	:effect (and
		;; For every state ?s
			(and
				(not (gripper_empty ?v))
				(not (object_at ?o ?wp))
				(holding ?v ?o)
			)
	)
)

(:action putdown_object
	:parameters (?v - robot ?wp ?near_wp - waypoint ?o - object)
	:precondition (and
		(robot_at ?v ?near_wp)
		(near_for_grasping ?near_wp ?wp)
		(holding ?v ?o)
	)
	:effect (and
		;; For every state ?s
			(and
				(not (holding ?v ?o))
				(gripper_empty ?v)
				(object_at ?o ?wp)
			)
	)
)

(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and
		(robot_at ?v ?from)
	)
	:effect (and
		;; For every state ?s
			(and
				(not (robot_at ?v ?from))
				(robot_at ?v ?to)
			)
	)
)

(:action push_object
	:parameters (?v - robot ?ob - object ?t - type ?from ?to ?near_wp - waypoint)
	:precondition (and
		(robot_at ?v ?near_wp)
		(object_at ?ob ?from)
		(is_of_type ?ob ?t)
		(can_push ?v ?t)
		(near_for_pushing ?near_wp ?from)
	)
	:effect (and
		;; For every state ?s
		(not (robot_at ?v ?from))
		(not (object_at ?ob ?from))
		(robot_at ?v ?to)
		(object_at ?ob ?to)
	)
)

(:action tidy_object
	:parameters (?v - robot ?o - object ?b - box ?t - type)
	:precondition (and
		(is_of_type ?o ?t)
		(inside ?o ?b)
		(can_fit_inside ?t ?b)
	)
	:effect (and
		;; For every state ?s
			(and
				(tidy ?o)
			)
		)
)

)
