cmake_minimum_required(VERSION 2.8)
project (STAM)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules/ )

# The version number.
#set (Tutorial_VERSION_MAJOR 1)
#set (Tutorial_VERSION_MINOR 0)
 
include_directories("${PROJECT_SOURCE_DIR}/src/include")

#########################################################
# FIND OPENCV
#########################################################
find_package( OpenCV REQUIRED )

find_package(cvsba REQUIRED )

########################################################
# Linking & stuff
#########################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(STAM_INC_DIR "include")
set(STAM_SRC_DIR "src")

include_directories( ${STAM_INC_DIR} )

set(STAM_INC "${STAM_INC_DIR}/STAM.h"
               "${STAM_INC_DIR}/types.h"
               "${STAM_INC_DIR}/utils.h"
               "${STAM_INC_DIR}/VideoSource.h"

    )

set(STAM_SRC "${STAM_SRC_DIR}/STAM.cpp"
               "${STAM_SRC_DIR}/types.cpp"
               "${STAM_SRC_DIR}/utils.cpp"
               "${STAM_SRC_DIR}/VideoSource.cpp"

   )

add_library( stam ${STAM_INC} ${STAM_SRC})

add_executable( demo_level3 "${STAM_SRC_DIR}/main.cpp" )

add_executable(cvsba_simple_test src/cvsba_simple_test.cpp)

target_link_libraries(demo_level3 ${OpenCV_LIBS} ${cvsba_LIBS} stam)

target_link_libraries(cvsba_simple_test ${OpenCV_LIBS} ${cvsba_LIBS})

target_link_libraries( demo_level3 ${OpenCV_LIBS} ${cvsba_LIBS})





