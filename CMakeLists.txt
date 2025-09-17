cmake_minimum_required(VERSION 3.8.0)
project(argos3_new_robots)

# Set the compilation type
if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel ..." FORCE)
endif(CMAKE_BUILD_TYPE STREQUAL "")

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts (if needed)
#Unnecessary for now as we have no environment variables to be set and no additional cmake files are used. 
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(ARGoS REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

# Look for the (optional) galib library
find_package(GALIB)
if(GALIB_FOUND)
  include_directories(${GALIB_INCLUDE_DIRS})
endif(GALIB_FOUND)

#
# Check for Lua 5.3
#
find_package(Lua)
if(LUA_FOUND)
  if(${LUA_VERSION_STRING} VERSION_GREATER_EQUAL "5.3")
    set(ARGOS_WITH_LUA ON)
    include_directories(AFTER ${LUA_INCLUDE_DIR})
  else()
    message(STATUS "Lua >=5.3 not found")
  endif()
else(LUA_FOUND)
  message(STATUS "Lua >=5.3 not found")
endif(LUA_FOUND)

# to use the new robot
add_subdirectory(argos3)

#Fix new Footbot plugin
# add_subdirectory(newfootbot)
