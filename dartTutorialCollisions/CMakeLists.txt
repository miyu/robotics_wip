cmake_minimum_required(VERSION 3.9)
project(dartTutorialCollisions)

set(CMAKE_CXX_STANDARD 17)

add_executable(dartTutorialCollisions main.cpp)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

target_link_libraries(dartTutorialCollisions dart dart-gui dart-utils-urdf ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system filesystem regex)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(dartTutorialCollisions ${Boost_LIBRARIES} assimp)
endif()