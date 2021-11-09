#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project.
PROJECT(roviproject)

add_compile_options(-std=c++17)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_VERBOSE_MAKEFILE false) 

# Now set the RW root
SET(RW_ROOT $ENV{RW_ROOT})
SET(RWS_ROOT $ENV{RW_ROOT})


# We use the settings that robwork studio uses
IF( NOT DEFINED CMAKE_BUILD_TYPE )
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#Use RobWork and RobWorkSim

FIND_PACKAGE(RobWork REQUIRED)
FIND_PACKAGE(RobWorkStudio REQUIRED)
find_package(OpenCV REQUIRED)


SET(RobWork_DIR ${RW_ROOT}/cmake) 
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

SET(RobWorkStudio_DIR ${RWS_ROOT}/cmake) 
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS}) 
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS})


#Include RobWorkSim settings

SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${RWS_ROOT}/build ${CMAKE_MODULE_PATH})

# Set the output dir for generated libraries and binaries


SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)


# openmp is used to parallize some for-loops in some code 
FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ENDIF()

file(GLOB SOURCES src/*.cpp)


# LIBRARIES ---------------------------------------------------------------------------------------------------------------------------------------------
add_executable(Interpolation ${SOURCES})
target_link_libraries(Interpolation ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${OpenCV_LIBS})



