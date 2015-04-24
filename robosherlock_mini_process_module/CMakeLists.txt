cmake_minimum_required(VERSION 2.8.3)
project(robosherlock_mini_process_module)

find_package(catkin REQUIRED COMPONENTS
  roslisp
  alexandria
  cram_process_modules
  cram_roslisp_common
  cram_designators
  cl_transforms
  cram_roslisp_common
  cram_designators
  cram_uima)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
