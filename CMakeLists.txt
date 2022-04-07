cmake_minimum_required(VERSION 2.8)
# set(CMAKE_BUILD_TYPE "Release")

project(steroVision)
set(CMAKE_BUILD_TYPE "Debug")
#设置使用C++11编译
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# elseif(COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# else()
#     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()

#导致断点乱跳？
# set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)

# SET(CMAKE_CXX_COMPILER "g++")
# SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
# set(CMAKE_CXX_STANDARD 11)
#设定可执行二进制文件的目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
	
#设定存放编译出来的库文件的目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 
#并且把该目录设为连接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)

# find_package(Pangolin REQUIRED)
# include_directories(${Pangolin_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# # 添加g2o的依赖
# # 因为g2o不是常用库，要添加它的findg2o.cmake文件
# LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# SET( G2O_ROOT /usr/local/include/g2o )
# FIND_PACKAGE( G2O )
# CSparse
FIND_PACKAGE( CSparse )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

#add library
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)

# #add executable

# add_executable(pointcloud src/pointcloud.cpp)
# target_link_libraries(pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES})

# add_executable(test01 src/test01.cpp)
# target_link_libraries(test01 ${OpenCV_LIBS})

add_executable(colorfulPoint src/colorfulPoint.cpp)
target_link_libraries(colorfulPoint ${OpenCV_LIBS})

# ADD_EXECUTABLE( generatePointCloud src/generatePointCloud.cpp )
# TARGET_LINK_LIBRARIES( generatePointCloud 
#     ${OpenCV_LIBS} 
#     ${PCL_LIBRARIES} )