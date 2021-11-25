cmake_minimum_required(VERSION 3.0.2)
project(robot_sim)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

set(CHOREONOID_SKIP_QT_CONFIG true)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  choreonoid
  message_generation
  )

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++14)
endif()

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src)