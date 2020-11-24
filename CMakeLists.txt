cmake_minimum_required(VERSION 3.0)
project(ethercat_device_configurator)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
endif()

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror)
add_definitions(-DMELO_USE_COUT)


find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED)

find_package(anydrive )
find_package(elmo_ethercat_sdk )
find_package(maxon_epos_ethercat_sdk )
find_package(rokubi_rsl_ethercat_sdk )

if(anydrive_FOUND)
  add_definitions(-D_ANYDRIVE_FOUND_)
  set(PACKAGE_DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
    anydrive
  )
endif()
if(rokubi_rsl_ethercat_sdk_FOUND)
  add_definitions(-D_ROKUBI_FOUND_)
  set(PACKAGE_DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
    rokubi_rsl_ethercat_sdk
  )
endif()
if(elmo_ethercat_sdk_FOUND)
  add_definitions(-D_ELMO_FOUND_)
  set(PACKAGE_DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
    elmo_ethercat_sdk
  )
endif()

if(maxon_epos_ethercat_sdk_FOUND)
  add_definitions(-D_MAXON_FOUND_)
  set(PACKAGE_DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
    maxon_epos_ethercat_sdk
  )
endif()

foreach(sdk ${PACKAGE_DEPENDENCIES})
  message("Found EtherCAT device sdk: " ${sdk})
endforeach()

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
  ${anydrive_INCLUDE_DIRS}
  ${elmo_ethercat_sdk_INCLUDE_DIRS}
  ${maxon_epos_ethercat_sdk_INCLUDE_DIRS}
  ${rokubi_rsl_ethercat_sdk_INCLUDE_DIRS}
)


file(GLOB_RECURSE SRC_FILES ./src/*.c*)

#set(SRC_FILES
#    src/anydrive/Anydrive.cpp)

add_library(${PROJECT_NAME}
  ${SRC_FILES}

)
#add_dependencies(
#  ${PROJECT_NAME}
#  ${anydrive_EXPORTED_TARGETS}
#  ${rokubi_rsl_ethercat_sdk_EXPORTED_TARGETS}
#  ${elmo_ethercat_sdk_EXPORTED_TARGETS}
#)
target_link_libraries(
  ${PROJECT_NAME}
  ${anydrive_LIBRARIES}
  ${elmo_ethercat_sdk_LIBRARIES}
  ${maxon_epos_ethercat_sdk_LIBRARIES}
  ${rokubi_rsl_ethercat_sdk_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  stdc++fs
)



add_executable(
  standalone
  src/standalone.cpp
)
add_dependencies(
  standalone
  ${PROJECT_NAME}
  ${anydrive_EXPORTED_TARGETS}
  ${rokubi_rsl_ethercat_sdk_EXPORTED_TARGETS}
  ${elmo_ethercat_sdk_EXPORTED_TARGETS}
  ${maxon_epos_ethercat_sdk_EXPORTED_TARGETS}
)
target_link_libraries(
  standalone
  ${PROJECT_NAME}
  ${anydrive_LIBRARIES}
  ${elmo_ethercat_sdk_LIBRARIES}
  ${maxon_epos_ethercat_sdk_LIBRARIES}
  ${rokubi_rsl_ethercat_sdk_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  #Eigen3:Eigen
  -pthread
  stdc++fs
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
