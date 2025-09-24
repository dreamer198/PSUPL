cmake_minimum_required(VERSION 3.16)
project(pc_ransac LANGUAGES CXX VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
        message(FATAL_ERROR "GCC version must be at least 9.0 for advanced template support!")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.28")
        message(FATAL_ERROR "MSVC version must be at least 19.28 (Visual Studio 2019 16.8)!")
    endif()
endif()

set(CMAKE_CXX_FLAGS "-fpermissive -fopenmp -mavx2 -mfma")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native -flto")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fsanitize=address -fsanitize=undefined")

find_package(OpenMP 4.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLFW3 3.3 REQUIRED)
find_package(VTK 9.0 REQUIRED COMPONENTS CommonCore CommonDataModel FiltersCore)

find_package(PCL 1.12 REQUIRED COMPONENTS common io filters features surface)

pkg_check_modules(NLOPT REQUIRED nlopt>=2.7)
pkg_check_modules(CERES REQUIRED ceres-solver>=2.1)

find_package(TBB REQUIRED)

include_directories(./)
include_directories(./MiscLib/)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

FILE(GLOB SRC "./*.cpp")
FILE(GLOB MISC_SRC "./MiscLib/*.cpp")

add_library(libransac STATIC ${SRC} ${MISC_SRC})

set_target_properties(libransac PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(libransac 
    PUBLIC
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    Eigen3::Eigen
    ${BLAS_LIBRARIES}
    ${LAPACK_LIBRARIES}
    OpenMP::OpenMP_CXX
    Threads::Threads
    ${NLOPT_LIBRARIES}
    ${CERES_LIBRARIES}
    TBB::tbb
)

add_executable(main main.cpp)

target_link_libraries(main 
    libransac
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    glfw
)

install(TARGETS main libransac
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
