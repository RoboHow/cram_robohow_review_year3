cmake_minimum_required(VERSION 2.8.3)
project(cram_rhy3_pickandplace_demo)

find_package(catkin REQUIRED COMPONENTS
  alexandria
  bullet_reasoning
  bullet_reasoning_designators
  cram_beliefstate
  cram_designators
  cram_environment_representation
  cram_language
  cram_physics_utils
  cram_plan_knowledge
  cram_plan_library
  cram_pr2_knowledge
  cram_reasoning
  cram_roslisp_common
  designator_integration_msgs
  designators_ros
  location_costmap
  object_location_designators
  occupancy_grid_costmap
  point_head_process_module
  pr2_description_lowres
  pr2_manipulation_knowledge
  pr2_manipulation_process_module
  pr2_navigation_process_module
  pr2_reachability_costmap
  robosherlock_process_module
  roslisp
  roslisp_utilities
  semantic_map_costmap
  person_msgs)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
