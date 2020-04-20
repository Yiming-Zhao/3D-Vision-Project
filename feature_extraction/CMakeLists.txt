cmake_minimum_required(VERSION 2.8.11)

project(feature_extraction)


# or to require a specific version: find_package(COLMAP 3.4 REQUIRED)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


#COLMAP_FOUND
find_package(COLMAP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
if(COLMAP_FOUND)
include_directories(${COLMAP_INCLUDE_DIRS})
link_directories(${COLMAP_LINK_DIRS})


endif(COLMAP_FOUND)

#SQLite3
find_package (SQLite3)
if (SQLITE3_FOUND)
  include_directories(${SQLITE3_INCLUDE_DIRS})
  
endif (SQLITE3_FOUND)

#Eigen3 3.3.4
find_package(Eigen3 REQUIRED)
if (EIGEN3_FOUND)
include_directories(${EIGEN3_INCLUDE_DIR})

endif (EIGEN3_FOUND)

add_executable(feature_extraction feature_extraction.cpp)
target_link_libraries(feature_extraction ${COLMAP_LIBRARIES} ${SQLITE3_LIBRARIES} Eigen3::Eigen)
