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
     9: observe-classifiable_from pickup_wp object_wp object0 l3 l2 kid_0_kb
     10: ramificate
     11: finalise_classification object0 pickup_wp object_wp
     12: observe-is_of_type object0 dinosaur l4 l3 kid_0_kb
     13: ramificate
     14: pickup_object kenny object_wp pickup_wp object0 dinosaur
     15: observe-holding kenny object0 l5 l4 kid_0_kb
     16: ramificate
     17: goto_waypoint kenny pickup_wp idle_wp
     18: putdown_object kenny drop_wp idle_wp object0
     19: pop l5 l4
     20: ramificate
     21: push_object kenny object0 dinosaur object_wp drop_wp pickup_wp
     22: goto_waypoint kenny drop_wp idle_wp
     23: pop l4 l3
     24: ramificate
     25: emote kenny sound_no wiggle_error
     26: goto_waypoint kenny pickup_wp idle_wp
     27: jump 5
     28: ramificate
     29: emote kenny sound_hello wiggle_error
     30: goto_waypoint kenny pickup_wp idle_wp
     31: jump 5
     32: ramificate
     33: observe-has_commanded kid_0 links l2 l1 kid_0_kb
     34: ramificate
     35: next_turn kid_0 kid_1
     36: pop l1 l0
     37: ramificate
     38: observe-has_commanded kid_0 gone l3 l2 kid_0_kb
     39: ramificate
     40: goto_waypoint kenny idle_wp pickup_wp
     41: emote kenny sound_no wiggle_idle
     42: goto_waypoint kenny pickup_wp idle_wp
     43: jump 5
     44: ramificate
     45: jump 5
     46: shed_knowledge kid_0_kb basis_kb

time spent:    0.00 seconds
