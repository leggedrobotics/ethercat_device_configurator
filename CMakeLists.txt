cmake_minimum_required(VERSION 3.0)
project(ethercat_device_configurator)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror)


find_package(yaml-cpp REQUIRED)

set(PACKAGE_DEPENDENCIES
  anydrive
  elmo_ethercat_sdk
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)


file(GLOB_RECURSE SRC_FILES ./src/*.c*)

#set(SRC_FILES
#    src/anydrive/AnydriveEthercatSlave.cpp)

add_library(${PROJECT_NAME}
  ${SRC_FILES}

)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)



add_executable(
  standalone
  src/standalone.cpp
)
add_dependencies(
  standalone
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  standalone
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  #Eigen3:Eigen
  -pthread
)

install(TARGETS ${PROJECT_NAME} standalone
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#if(CATKIN_ENABLE_TESTING)
  # Tests in order of their dependency.
#  catkin_add_gtest(
#    ${PROJECT_NAME}_test
#    test/VersionTest.cpp
#    test/${PROJECT_NAME}_test.cpp
#  )
#endif()

#if(TARGET ${PROJECT_NAME}_test)
#  target_link_libraries(
#    ${PROJECT_NAME}_test
#    ${PROJECT_NAME}
#  )
#endif()
