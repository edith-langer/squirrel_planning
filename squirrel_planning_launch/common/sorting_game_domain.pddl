(define (domain classify_objects)
(:requirements :typing :conditional-effects :negative-preconditions :disjunctive-preconditions)

(:types
	waypoint robot object
	kid
	level
	state
	knowledgebase
	wiggle sound command
	type
	jump-type
	state
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint ?s - state)
	(object_at ?o - object ?wp - waypoint ?s - state)
	(classified ?o - object ?s - state)
	(classification_failed ?o - object ?s - state)
	(classifiable_from ?from - waypoint ?view - waypoint ?o - object ?s - state)

	(gripper_empty ?v - robot ?s - state)
	(holding ?v - robot ?o - object ?s - state)
	(near ?wp1 ?wp2 - waypoint)
	(current_turn ?k - kid ?s - state)
	(has_commanded ?k - kid ?c - command ?s - state)

	(part-of ?s - state ?kb - knowledgebase)
	(current_kb ?kb - knowledgebase)
	(parent ?kb ?kb2 - knowledgebase)
	(is_of_type ?o - object ?t - type ?s - state)

	;; Bookkeeping predicates.
	(next ?l ?l2 - level)
	(lev ?l - LEVEL)
	(resolve-axioms)
)

(:constants
	basis_kb - knowledgebase
	dummy_state - state
)

(:action next_turn
	:parameters (?k1 ?k2 - kid)
	:precondition (and

	)
	:effect (and
		
	)
)

(:action pickup_object
	:parameters (?v - robot ?wp ?near_wp - waypoint ?ob - object ?t - type)
	:precondition (and
		(robot_at ?v ?near_wp dummy_state)
		(object_at ?ob ?wp dummy_state)
		(gripper_empty ?v dummy_state)
		(is_of_type ?ob ?t dummy_state)
		(near ?near_wp ?wp)
	)
	:effect (and
		
	)
)

(:action putdown_object
	:parameters (?v - robot ?wp ?near_wp - waypoint ?o - object ?t - type)
	:precondition (and
		(robot_at ?v ?near_wp dummy_state)
		(object_at ?o ?wp dummy_state)
		(gripper_empty ?v dummy_state)
		(is_of_type ?o ?t dummy_state)
		(near ?near_wp ?wp)
	)
	:effect (and
		
	)
)

(:action push_object
	:parameters (?v - robot ?ob - object ?t - type ?from ?to ?near_wp - waypoint)
	:precondition (and
		(robot_at ?v ?near_wp dummy_state)
		(object_at ?ob ?from dummy_state)
		(is_of_type ?ob ?t dummy_state)
		(near ?near_wp ?from)
	)
	:effect (and
		
	)
)

(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and
		(not (resolve-axioms))
		(robot_at ?v ?from dummy_state)
	)
	:effect (and
		
	)
)

(:action emote
	:parameters (?v - robot ?s - sound ?w - wiggle)
	:precondition (and )
	:effect (and )
)

;; Attempt to classify the object.
(:action observe-classifiable_from
	:parameters (?from ?view - waypoint ?o - object ?v - robot  ?l ?l2 - level ?kb - knowledgebase)
	:precondition (and
		(not (resolve-axioms))
		(next ?l ?l2)
		(lev ?l)
		(current_kb ?kb)
		(not (current_kb basis_kb))

		(robot_at ?v ?from dummy_state)
	)
	:effect (and
		
	)
)

(:action observe-holding
	:parameters (?v - robot ?o - object ?l ?l2 - level ?kb - knowledgebase)
	:precondition (and
		(not (resolve-axioms))
		(next ?l ?l2)
		(lev ?l)
		(current_kb ?kb)
		(not (current_kb basis_kb))
	)
	:effect (and
		
	)
)

(:action observe-has_commanded
	:parameters (?k - kid ?c - command ?l ?l2 - level ?kb - knowledgebase)
	:precondition (and
		(not (resolve-axioms))
		(next ?l ?l2)
		(lev ?l)
		(current_kb ?kb)
		(not (current_kb basis_kb))
	)
	:effect (and
		
	)
)

(:action observe-is_of_type
	:parameters (?o - object ?t - type ?l ?l2 - level ?kb - knowledgebase)
	:precondition (and
		(not (resolve-axioms))
		(next ?l ?l2)
		(lev ?l)
		(current_kb ?kb)
		(not (current_kb basis_kb))
	)
	:effect (and
		
	)
)

(:action observe-current_turn
	:parameters (?k - kid ?l ?l2 - level ?kb - knowledgebase)
	:precondition (and
		(not (resolve-axioms))
		(next ?l ?l2)
		(lev ?l)
		(current_kb ?kb)
		(not (current_kb basis_kb))
	)
	:effect (and
		
	)
)

(:action finalise_classification
	:parameters (?ob - object ?from ?view - waypoint)
	:precondition (and
		(not (resolve-axioms))
	)
	:effect (and

	)
)

;; Exit the current branch.
(:action pop
	:parameters (?l ?l2 - level)
	:precondition (and
		(lev ?l)
		(next ?l2 ?l)
		(not (resolve-axioms))
	)
	:effect (and 
		
	)
)

;; Resolve the axioms manually.
(:action ramificate
	:parameters ()
	:precondition (resolve-axioms)
	:effect (and 

	)
)
;; Move 'down' into the knowledge base.
(:action assume_knowledge
	:parameters (?old_kb ?new_kb - knowledgebase)
	:precondition (and
		(not (resolve-axioms))
		(current_kb ?old_kb)
		(parent ?old_kb ?new_kb)
	)
	:effect (and

	)
)
;; Move 'up' into the knowledge base.
(:action shed_knowledge
	:parameters (?old_kb ?new_kb - knowledgebase)
	:precondition (and
		(not (resolve-axioms))
		(current_kb ?old_kb)
	)
	:effect (and
		
	)
)


(:action jump
	:parameters (?j - jump-type)
	:precondition (and )
	:effect (and )
)

)
