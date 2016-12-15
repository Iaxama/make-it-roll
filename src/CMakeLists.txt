add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME}
        ${YARP_LIBRARIES}
        ctrlLib
        )


install (TARGETS ${PROJECT_NAME} DESTINATION bin)