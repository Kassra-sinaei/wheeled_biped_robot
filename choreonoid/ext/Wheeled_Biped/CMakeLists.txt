set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

set (sources
    sources/robot.cpp
    sources/controller.cpp
)
set(headers
    headers/robot.h
    headers/controller.h
)

add_cnoid_simple_controller(WBFull main.cpp ${sources}  ${headers})
add_cnoid_simple_controller(WB1MinimumController minimum.cpp)