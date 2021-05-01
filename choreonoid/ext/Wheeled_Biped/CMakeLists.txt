set (sources
    sources/robot.cpp
    sources/controller.cpp
)
set(headers
    headers/robot.h
    headers/controller.h
)

#add_cnoid_simple_controller(SR1_byebye byebye.cpp ${sources}  ${headers})
add_cnoid_simple_controller(WB1MinimumController minimum.cpp)