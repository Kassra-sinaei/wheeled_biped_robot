#set (sources
    #sources/hand_trajectory.cpp
#)
#set(headers
    #headers/hand_trajectory.h
#)

#add_cnoid_simple_controller(SR1_byebye byebye.cpp ${sources}  ${headers})
add_cnoid_simple_controller(WB1MinimumController minimum.cpp)