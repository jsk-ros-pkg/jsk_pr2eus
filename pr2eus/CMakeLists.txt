cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#ADD_CUSTOM_COMMAND(
#  OUTPUT ${CMAKE_SOURCE_DIR}/pr2.dae
#  COMMAND rosrun collada_urdf_jsk_patch urdf_to_collada `rospack find pr2_mechanism_model`/pr2.urdf ${CMAKE_SOURCE_DIR}/pr2.dae)
#ADD_CUSTOM_COMMAND(
#  OUTPUT ${CMAKE_SOURCE_DIR}/pr2.l
#  COMMAND rosrun euscollada collada2eus ${CMAKE_SOURCE_DIR}/pr2.dae `rospack find euscollada`/pr2.yaml ${CMAKE_SOURCE_DIR}/pr2.l DEPENDS pr2.dae)
ADD_CUSTOM_COMMAND(
  OUTPUT ${CMAKE_SOURCE_DIR}/pr2.l
  COMMAND rosrun roseus roseus `rospack find pr2eus`/make-pr2-model-file.l           "\"(ros::roseus" "\\\"make-pr2-model-file\\\")\"" "\"(make-pr2-model-file" ":output-directory" "(ros::rospack-find" "\\\"pr2eus\\\"))\"" "\"(exit" "0)\""
  )
execute_process(COMMAND rosparam get /robot_description ERROR_VARIABLE _roscore_output OUTPUT_VARIABLE _roscore_output RESULT_VARIABLE _roscore_failed)
if(_roscore_failed)
  message("[pr2eus] /robot_description is required for make pr2.l model file")
else(_roscore_failed)
  ADD_CUSTOM_TARGET(pr2model ALL DEPENDS ${CMAKE_SOURCE_DIR}/pr2.l)
endif(_roscore_failed)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_rostest(test/pr2eus-test.launch)
rosbuild_add_rostest(test/make-pr2-model-file-test.launch)
