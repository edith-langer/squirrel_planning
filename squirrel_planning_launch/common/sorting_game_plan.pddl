ff: found legal plan as follows

step    0: ramificate
     1: emote kenny sound_hello wiggle_no
     2: goto_waypoint kenny start_wp idle_wp
     3: assume_kb basis_kb kid_0_kb
     4: ramificate
     5: observe-has_commanded kid_0 gehe l2 l1 kid_0_kb
     6: ramificate
     7: emote kenny sound_yeah wiggle_idle
     8: goto_waypoint kenny idle_wp pickup_wp
     9: observe-classifiable_from pickup_wp object_wp object1 l3 l2 kid_0_kb
     10: ramificate
     11: finalise_classification object1 pickup_wp object_wp
     12: observe-is_of_type object1 dinosaur l4 l3 kid_0_kb
     13: ramificate
     14: goto_waypoint kenny pickup_wp grasp_wp
     15: pickup_object kenny object_wp grasp_wp object1 dinosaur
     16: observe-holding kenny object1 l5 l4 kid_0_kb
     17: ramificate
     18: goto_waypoint kenny grasp_wp idle_wp
     19: putdown_object kenny drop_wp idle_wp object1
     20: pop l5 l4
     21: ramificate
     22: goto_waypoint kenny pickup_wp push_wp
     23: push_object kenny object1 dinosaur object_wp drop_wp push_wp
     24: goto_waypoint kenny drop_wp idle_wp
     25: pop l4 l3
     26: ramificate
     27: emote kenny sound_no wiggle_error
     28: goto_waypoint kenny pickup_wp idle_wp
     29: jump 5
     30: ramificate
     31: emote kenny sound_hello wiggle_error
     32: goto_waypoint kenny pickup_wp idle_wp
     33: jump 5
     34: ramificate
     35: observe-has_commanded kid_0 links l2 l1 kid_0_kb
     36: ramificate
     37: next_turn kid_0 kid_1
     38: pop l1 l0
     39: ramificate
     40: observe-has_commanded kid_0 gone l3 l2 kid_0_kb
     41: ramificate
     42: goto_waypoint kenny idle_wp pickup_wp
     43: emote kenny sound_no wiggle_idle
     44: goto_waypoint kenny pickup_wp idle_wp
     45: jump 5
     46: ramificate
     47: jump 5
     48: shed_knowledge kid_0_kb basis_kb

time spent:    0.00 seconds
