
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION applications)
