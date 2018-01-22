cmake_minimum_required(VERSION 2.4.6)

set(ROS_ROOT $ENV{ROS_ROOT})
set(ROS_ROOT_DEFINED FALSE)
if(DEFINED ROS_ROOT)
  message("ROS detected.")
  set(ROS_ROOT_DEFINED TRUE)
endif(DEFINED ROS_ROOT)

set(BUILD_WITH_ROS ${ROS_ROOT_DEFINED} CACHE BOOL "Build with ROS support.")

if(BUILD_WITH_ROS AND NOT ROS_ROOT_DEFINED)
  message(FATAL_ERROR "ROS support is selected, but ROS_ROOT is not set.")
endif(BUILD_WITH_ROS AND NOT ROS_ROOT_DEFINED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(XBOW_LINK_LIBS "")

if(BUILD_WITH_ROS)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  # Set the build type.  Options are:
  #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
  #  Debug          : w/ debug symbols, w/o optimization
  #  Release        : w/o debug symbols, w/ optimization
  #  RelWithDebInfo : w/ debug symbols, w/ optimization
  #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
  set(ROS_BUILD_TYPE RelWithDebInfo CACHE STRING "Built type for ROS")
  rosbuild_init()

  #uncomment if you have defined messages
  #rosbuild_genmsg()
  #uncomment if you have defined services
  #rosbuild_gensrv()
else(BUILD_WITH_ROS)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Built type for cmake")
  # Find serial, if it hasn't already been found
  find_package(serial REQUIRED)

  if(serial_FOUND)
    include_directories(${serial_INCLUDE_DIRS})
    list(APPEND XBOW_LINK_LIBS ${serial_LIBRARIES})
  else(serial_FOUND)
    message(FATAL_ERROR "Serial library was not found.")
  endif(serial_FOUND)
endif(BUILD_WITH_ROS)

# Add the include folder to the include path
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add default source files
set(XBOW_SRCS src/xbow440.cpp)
# Add default header files
set(XBOW_HEADERS include/xbow440/xbow440.h)

# Find Boost, if it hasn't already been found
find_package(Boost COMPONENTS thread system REQUIRED)

link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

list(APPEND XBOW_LINK_LIBS ${Boost_SYSTEM_LIBRARY}
                           ${Boost_THREAD_LIBRARY})

## Build the xbow Library

# Compile the Library
if(BUILD_WITH_ROS)
  rosbuild_add_library(xbow440 ${XBOW_SRCS} ${XBOW_HEADERS})
else(BUILD_WITH_ROS)
  add_library(xbow440 ${XBOW_SRCS} ${XBOW_HEADERS})
endif(BUILD_WITH_ROS)
target_link_libraries(xbow440 ${XBOW_LINK_LIBS})

## Build the ROS wrapper

if(BUILD_WITH_ROS)
  rosbuild_add_executable(xbow440_node src/xbow440_node.cc)
  target_link_libraries(xbow440_node xbow440)
endif(BUILD_WITH_ROS)

## Build Examples

# If asked to
IF(XBOW440_BUILD_EXAMPLES)
    # Compile the atrv Test program
    add_executable(xbow440_example examples/xbow440_example.cpp)
    # Link the Test program to the atrv library
    target_link_libraries(xbow440_example xbow440)
ENDIF(XBOW440_BUILD_EXAMPLES)

## Build tests

# If asked to
IF(XBOW440_BUILD_TESTS)
    # Find Google Test
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    # Compile the atrv Test program
    add_executable(xbow_tests tests/xbow_tests.cpp)
    # Link the Test program to the atrv library
    target_link_libraries(xbow_tests ${GTEST_BOTH_LIBRARIES}
                          xbow440)

    add_test(AllTestsIntest_xbow xbow_tests)
ENDIF(XBOW440_BUILD_TESTS)

## Setup install and uninstall

# Unless asked not to...
IF(NOT XBOW_DONT_CONFIGURE_INSTALL AND NOT BUILD_WITH_ROS)
    # Configure make install
    IF(NOT CMAKE_INSTALL_PREFIX)
        SET(CMAKE_INSTALL_PREFIX /usr/local)
    ENDIF(NOT CMAKE_INSTALL_PREFIX)
    
    INSTALL(TARGETS xbow440
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
    )
    
    INSTALL(FILES include/xbow440/Xbow440.h
            DESTINATION include/xbow440)
    
    IF(NOT CMAKE_FIND_INSTALL_PATH)
        set(CMAKE_FIND_INSTALL_PATH ${CMAKE_ROOT})
    ENDIF(NOT CMAKE_FIND_INSTALL_PATH)
    
    # INSTALL(FILES Findatrv.cmake DESTINATION ${CMAKE_FIND_INSTALL_PATH}/Modules/)
    
    ADD_CUSTOM_TARGET(uninstall @echo uninstall package)
    
    IF (UNIX)
      ADD_CUSTOM_COMMAND(
        COMMENT "uninstall package"
        COMMAND xargs ARGS rm < install_manifest.txt
        
        TARGET  uninstall
      )
    ELSE(UNIX)
      ADD_CUSTOM_COMMAND(
        COMMENT "uninstall only implemented in unix"
        TARGET  uninstall
      )
    ENDIF(UNIX)
ENDIF(NOT XBOW_DONT_CONFIGURE_INSTALL AND NOT BUILD_WITH_ROS)

