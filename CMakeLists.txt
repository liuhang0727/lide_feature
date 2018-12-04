project(lide_feature)
cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_definitions(${PCL_DIFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})

find_package(libLAS REQUIRED)

add_executable(lide_feature main.cpp lide_feature.hpp)
target_link_libraries(lide_feature ${PCL_LIBRARIES} ${libLAS_LIBRARIES})
