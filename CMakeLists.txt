cmake_minimum_required(VERSION 3.0.0)
project(gps_call)

IF(NOT CMAKE_BUILD_TYPE)
    # SET(CMAKE_BUILD_TYPE Debug)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-reorder -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder  -O3")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Find Eigen3 Library
# Since it is header-only and it won't be linked,
# we have to explicitly add the include directories.
find_package(Eigen3 3.2 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        cmake_modules
        eigen_conversions
        std_msgs
        sensor_msgs
        geometry_msgs
        cv_bridge
#        message_filters
        tf_conversions
        tf
        )

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf
        #  DEPENDS system_lib
)


add_executable(gps_conversion_node
        gps_call_node.cpp
        gps_conversion.cpp
        CameraPose.cpp
        )

target_link_libraries(gps_conversion_node
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBS}
        )
