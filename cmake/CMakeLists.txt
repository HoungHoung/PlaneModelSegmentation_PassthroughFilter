cmake_minimum_required(VERSION 3.16.0)

project(PlaneModelSegmentation_PassthroughFilter)

find_package(PCL 1.8.1 REQUIRED)
find_package(VTK 8.2 REQUIRED)

if(MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "/Z7")
    add_definitions(-DNDEBUG)
    add_definitions(-D_CONSOLE)
    add_definitions(-D_SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING)
endif()

INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} )
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS} )
INCLUDE_DIRECTORIES(${VTK_INCLUDE_DIRS} )
LINK_DIRECTORIES(${VTK_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS} )
add_executable (src "main.cpp")
target_link_libraries (main ${PCL_LIBRARIES} ${VTK_LIBS})