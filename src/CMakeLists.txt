include_directories(${YARP_INCLUDE_DIRS})

add_library(CtrlModule
        CtrlModule.cpp include/CtrlModule.h)
#target_link_libraries(CtrlModule ${YARP_LIBRARIES})

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} CtrlModule ${YARP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION /usr/local/src/robot/iCubContrib/bin)
