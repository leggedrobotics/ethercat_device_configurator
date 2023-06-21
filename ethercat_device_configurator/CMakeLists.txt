# Copyright 2021 Robotic Systems Lab - ETH Zurich:
# Lennart Nachtigall, Jonas Junger
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


cmake_minimum_required(VERSION 3.16)
project(ethercat_device_configurator)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
endif()

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror -Wextra)
add_definitions(-DMELO_USE_COUT)


find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ethercat_sdk_master REQUIRED)

find_package(anydrive_rsl QUIET)
find_package(elmo_ethercat_sdk QUIET)
find_package(mps_ethercat_sdk QUIET)
find_package(maxon_epos_ethercat_sdk QUIET)
find_package(rokubimini_rsl_ethercat_slave QUIET)

if(anydrive_rsl_FOUND)
  add_definitions(-D_ANYDRIVE_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES anydrive_rsl)
endif()
if(rokubimini_rsl_ethercat_slave_FOUND)
  add_definitions(-D_ROKUBI_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES rokubimini_rsl_ethercat_slave)
endif()
if(elmo_ethercat_sdk_FOUND)
  add_definitions(-D_ELMO_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES elmo_ethercat_sdk)
endif()
if(mps_ethercat_sdk_FOUND)
  add_definitions(-D_MPSDRIVE_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES mps_ethercat_sdk)
endif()
if(maxon_epos_ethercat_sdk_FOUND)
  add_definitions(-D_MAXON_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES maxon_epos_ethercat_sdk)
endif()

foreach(sdk ${PACKAGE_DEPENDENCIES})
  message("Found EtherCAT device sdk: " ${sdk})
endforeach()

LIST(APPEND PACKAGE_DEPENDENCIES ament_cmake ethercat_sdk_master yaml-cpp)

file(GLOB_RECURSE SRC_FILES ./src/*.c*)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} stdc++fs yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})



add_executable(
  standalone
  src/standalone.cpp
)

target_link_libraries(
    standalone
    ${PROJECT_NAME}
    -pthread
)

install(
    TARGETS ${PROJECT_NAME} standalone
    EXPORT ${PROJECT_NAME}_export
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    DIRECTORY include/
    DESTINATION include
)
install( #requries rebuild if changes made in src folder --> enable sym-link install in colcon.
    DIRECTORY example_config/
    DESTINATION bin
)

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
# ament_export_include_directories(include)


ament_package()



