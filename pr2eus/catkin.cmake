cmake_minimum_required(VERSION 2.8.3)
project(pr2eus)

find_package(catkin REQUIRED COMPONENTS rostest)

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

install(DIRECTORY .
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*.l"
  PATTERN ".svn" EXCLUDE
  )

add_rostest(test/pr2eus-test.launch)
add_rostest(test/make-pr2-model-file-test.launch)
add_rostest(test/pr2-ri-test.launch)
