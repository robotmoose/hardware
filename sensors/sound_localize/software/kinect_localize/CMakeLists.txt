cmake_minimum_required (VERSION 2.8)
project(Sound_Localize_Kinect)

# Require C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(include)

# Add freenect libraries and required fix for Kinect Model #1473
set(freenect_fix_SRCS src/ofxKinectExtras.cpp)
add_library(freenect_fix ${freenect_fix_SRCS})

# Add sources required for dsp
set(dsp_SRCS src/xcor_td.cpp)
add_library(dsp ${dsp_SRCS})

# Build the executable
add_executable(Sound_Localize_Kinect Sound_Localize_Kinect.cpp)
# Link against required libraries
target_link_libraries(Sound_Localize_Kinect freenect freenect_fix dsp GL glut pthread)
