cmake_minimum_required(VERSION 3.1)
project(gp_project)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         ON)

find_package(LIBIGL REQUIRED QUIET)
find_package(pinocchio REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} SHARED
    src/fk.cpp
)


# Add your project files
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME} igl::core igl::opengl_glfw)

add_executable(bbw ./demos/bbw.cpp)
add_executable(bbw_cyl ./demos/bbw_cyl.cpp)
add_executable(tmp ./demos/tmp.cpp)
target_link_libraries(bbw ${PROJECT_NAME})
target_link_libraries(bbw_cyl ${PROJECT_NAME})
target_link_libraries(tmp igl::core igl::opengl_glfw igl::tetgen)

add_executable(fk_demo ./demos/fk.cpp)
target_link_libraries(fk_demo ${PROJECT_NAME})

