cmake_minimum_required(VERSION 2.8)
project (codeForLevel2)

# The version number.
#set (Tutorial_VERSION_MAJOR 1)
#set (Tutorial_VERSION_MINOR 0)
 
# configure a header file to pass some of the CMake settings
# to the source code
#configure_file (
#  "${PROJECT_SOURCE_DIR}/src/include/test.hpp"
#  "${PROJECT_BINARY_DIR}/test.hpp"
#  )
 
# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
#include_directories("${PROJECT_BINARY_DIR}" )
include_directories("${PROJECT_SOURCE_DIR}/src/include")


#########################################################
# FIND OPENCV
#########################################################
find_package( OpenCV REQUIRED )

#########################################################
# FIND PTHREADS
#########################################################
#find_package (Threads)


########################################################
# Linking & stuff
#########################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(LEVEL2_INC_DIR "include")
set(LEVEL2_SRC_DIR "src")

include_directories( ${LEVEL2_INC_DIR} )

set(LEVEL2_INC ismar.h "${LEVEL2_INC_DIR}/VOodoometry.h"
                       "${LEVEL2_INC_DIR}/types.h"
                       "${LEVEL2_INC_DIR}/utils.h"

    )

set(LEVEL2_SRC user_function.cpp "${LEVEL2_SRC_DIR}/VOodoometry.cpp"
                                 "${LEVEL2_SRC_DIR}/types.cpp"
                                 "${LEVEL2_SRC_DIR}/utils.cpp"

   )

add_executable( level2 main2.cpp ${LEVEL2_INC} ${LEVEL2_SRC})
add_executable( test_level2 src/joma3.cpp ${LEVEL2_INC} ${LEVEL2_SRC})
add_executable( demo_level2 main.cpp ${LEVEL2_INC} ${LEVEL2_SRC})

target_link_libraries( demo_level2 ${OpenCV_LIBS})
target_link_libraries( test_level2 ${OpenCV_LIBS})
target_link_libraries( level2 ${OpenCV_LIBS})

#${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT}  )





