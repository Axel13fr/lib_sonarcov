cmake_minimum_required(VERSION 3.1)
project(lib_sonarcov)

set(CMAKE_CXX_STANDARD 14)
# ext-numeric-literals is needed for Kinetic compilation
add_compile_options(-std=c++14 -Wall -Werror=return-type -fext-numeric-literals)

find_package(catkin REQUIRED COMPONENTS
    roscpp grid_map_core sensor_msgs)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sonarcov
  CATKIN_DEPENDS roscpp grid_map_core sensor_msgs
  DEPENDS Eigen3 Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

file(GLOB MANDA_SRC
    "src/*.cpp"
    "include/lib_sonarcov/*.h"
)


add_library(sonarcov SHARED ${MANDA_SRC})
target_link_libraries(sonarcov
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS sonarcov
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(./tests)
endif()
