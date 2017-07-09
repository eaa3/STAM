cmake_minimum_required(VERSION 2.8)
project (STAM)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/ )

# The version number.
#set (Tutorial_VERSION_MAJOR 1)
#set (Tutorial_VERSION_MINOR 0)
 
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src/include")

#########################################################
# FIND OPENCV AND SPARSE BUNDLE ADJUSTMENT PACKAGE
#########################################################
find_package( OpenCV REQUIRED )

find_package(cvsba REQUIRED )

find_package( Eigen3 REQUIRED ) # for rot matrix to quaternion conversion
include_directories( ${EIGEN3_INCLUDE_DIR} )

########################################################
# Linking & stuff
#########################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(STAM_INC "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(STAM_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

include_directories( ${STAM_INC} )

# set(VODOM_INC "${VIS_ODOM_INC}/STAM.h"
#                "${VIS_ODOM_INC}/types.h"
#                "${VIS_ODOM_INC}/utils.h"
#                "${VIS_ODOM_INC}/VideoSource.h"

#     )

set(STAM_LIBS "${STAM_SRC_DIR}/STAM.cpp"
                "${STAM_SRC_DIR}/cvsba_simple_test.cpp"
               "${STAM_SRC_DIR}/types.cpp"
               "${STAM_SRC_DIR}/utils.cpp"
               "${STAM_SRC_DIR}/VideoSource.cpp"

   )

add_library( vostam SHARED ${STAM_LIBS})

add_executable( demo ${STAM_SRC_DIR}/main.cpp )

# add_executable(cvsba_simple_test ${STAM_SRC_DIR}/cvsba_simple_test.cpp)
target_link_libraries(demo ${OpenCV_LIBS} ${cvsba_LIBS} vostam)

# target_link_libraries(cvsba_simple_test ${OpenCV_LIBS} ${cvsba_LIBS})

target_link_libraries(vostam ${OpenCV_LIBS} ${cvsba_LIBS} ${Eigen3_LIBS})

# target_link_libraries(demo ${OpenCV_LIBS} ${cvsba_LIBS})




