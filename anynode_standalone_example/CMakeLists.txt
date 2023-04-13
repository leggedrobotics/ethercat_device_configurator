cmake_minimum_required(VERSION 2.8.3)
project(anynode_standalone_example)

# Activate C++11.
set(CMAKE_CXX_FLAGS "-std=c++17")
add_definitions(-DMELO_FUNCTION_PRINTS)

set(PACKAGE_DEPENDENCIES
      any_node
      ethercat_device_configurator
)

find_package(catkin 
    REQUIRED COMPONENTS
      ${PACKAGE_DEPENDENCIES}
)

catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
      ${PACKAGE_DEPENDENCIES}
 DEPENDS 
 )
 
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
        src/AnyNodeStandaloneExample.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
) 

add_executable(${PROJECT_NAME}_node
        src/${PROJECT_NAME}_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
) 

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

