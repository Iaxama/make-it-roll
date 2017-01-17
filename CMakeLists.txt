project(make-it-roll)

cmake_minimum_required(VERSION 2.8.9)

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()
include_directories(src/include)

#Allowing linking between dynamic and static libraries
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

set(CMAKE_INSTALL_PREFIX ../install)

add_subdirectory(src)
add_subdirectory(app)
add_subdirectory(tests)

icubcontrib_add_uninstall_target()
