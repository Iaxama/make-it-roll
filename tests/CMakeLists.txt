#  RTF
#  Copyright: (C) 2015 Ali Paikan
#  Authors: Ali Paikan <ali.paikan@gmail.com>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#
project(rollingTest)

find_package(RTF)
find_package(RTF COMPONENTS DLL)

include_directories(${CMAKE_SOURCE_DIR}
        ${RTF_INCLUDE_DIRS}
        ${YARP_INCLUDE_DIRS}
        ${YARP_HELPERS_INCLUDE_DIR})

add_library(${PROJECT_NAME} MODULE
        RollingTest.cpp RollingTest.h)
target_link_libraries(${PROJECT_NAME}  ${RTF_LIBRARIES} CtrlModule)

install(TARGETS ${PROJECTNAME}
        EXPORT ${PROJECTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)

file(GLOB scripts fixtures/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_DATA_INSTALL_DIR}/fixtures)
