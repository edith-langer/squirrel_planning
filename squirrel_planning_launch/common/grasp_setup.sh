#!/bin/bash
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0
knowledge:
  knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'kenny'
  attribute_name: ''
  function_value: 0.0";
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0
knowledge:
  knowledge_type: 0
  instance_type: 'waypoint'
  instance_name: 'wp0'
  attribute_name: ''
  function_value: 0.0";
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0
knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'r', value: 'kenny'}
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0"
for i in 0 1 2 3 4 5
do
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0
knowledge:
  knowledge_type: 0
  instance_type: 'object'
  instance_name: 'obj$i'
  attribute_name: ''
  function_value: 0.0";
done;
rosservice call /kcl_rosplan/planning_server;
