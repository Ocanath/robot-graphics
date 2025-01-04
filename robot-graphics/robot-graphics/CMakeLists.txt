cmake_minimum_required(VERSION 3.10)

# Project Name
project(MyOpenGLProject VERSION 1.0 LANGUAGES CXX)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Source Files
set(SOURCES
	main.cpp
	args-parsing.cpp
	dh_hex_fixed.cpp
	dynahex.cpp
	finger-kinematics.cpp
	glfw_interface.cpp
	hexapod_footpath.cpp
	IIRsos.cpp
	kinematics.cpp
	kinematics_fixed.cpp
	log_fixed.cpp
	m_mcpy.cpp
	model.cpp
	objloader.cpp
	PPP.cpp
	psy-grips.cpp
	serial-thread.cpp
	sin_fast.cpp
	sin_lookup_table.cpp
	spatialAlgebra.cpp
	texture.cpp
	trig_fixed.cpp
	usb-com-handle.cpp
	utils.cpp
	vboindexer.cpp
	vect_fixed.cpp
)

# Include Directories
include_directories(external)
include_directories(external/assimp)

# Find OpenGL
find_package(OpenGL REQUIRED)

if (NOT OPENGL_FOUND)
    message(FATAL_ERROR "OpenGL not found!")
endif()

# Find GLEW (Optional if you're using it for OpenGL extensions)
find_package(GLEW REQUIRED)

if (NOT GLEW_FOUND)
    message(FATAL_ERROR "GLEW not found!")
endif()

# Find GLFW (Optional if you're using it for windowing and input handling)
find_package(glfw3 REQUIRED)

if (NOT glfw3_FOUND)
    message(FATAL_ERROR "GLFW not found!")
endif()

include_directories(${GLFW_INCLUDE_DIRS})

# Find Assimp
find_package(assimp REQUIRED)

if (NOT assimp_FOUND)
    message(FATAL_ERROR "Assimp not found!")
endif()


# Add Executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link Libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    OpenGL::GL
    GLEW::GLEW
    glfw
)

# Compiler Warnings (Optional)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Installation Instructions
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# Print a message for successful configuration
message(STATUS "CMake configuration successful. Build with `make`. Run the executable in the `bin` folder.")
