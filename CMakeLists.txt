cmake_minimum_required(VERSION 3.16)
project(finger_simulation)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(libigl)

igl_include(glfw)

add_executable(${PROJECT_NAME} 
    src/main.cpp
    src/rigging.cpp
    src/physics.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    igl::core
    igl::glfw
)
