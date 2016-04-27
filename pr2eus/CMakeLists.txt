cmake_minimum_required(VERSION 2.8.3)
project(pr2eus)

find_package(catkin REQUIRED COMPONENTS euscollada control_msgs nav_msgs dynamic_reconfigure rosgraph_msgs move_base_msgs pr2_msgs pr2_controllers_msgs sound_play
  roseus # this load roseus.cmake, so it needs to be located in the end
  )


catkin_package(
    DEPENDS 
    CATKIN-DEPENDS 
    INCLUDE_DIRS 
    LIBRARIES 
)

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

file(GLOB _install_files RELATIVE ${PROJECT_SOURCE_DIR} *.l *.launch)
install(FILES ${_install_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES pr2-send-joints.l pr2-read-state.l
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PERMISSIONS WORLD_EXECUTE
  )

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/pr2eus-test.launch)
  add_rostest(test/make-pr2-model-file-test.launch)
  #add_rostest(test/pr2-ri-test.launch)
  add_rostest(test/pr2-ri-test-simple.launch)
  add_rostest(test/robot-no-clock.test)
  add_rostest(test/default-ri-test.test)
  add_rostest(test/speak-test.test)
  add_rostest(test/robot-init-test.test)
endif()

generate_eusdoc(robot-interface.l)
generate_eusdoc(pr2-interface.l)
generate_eusdoc(pr2-utils.l)
generate_eusdoc(speak.l)
