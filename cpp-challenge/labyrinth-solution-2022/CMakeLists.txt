cmake_minimum_required(VERSION 3.10)

project(gsoc_labyrinth_2022)

set(CMAKE_CXX_STANDARD 11)

add_library(gsoc_labyrinth STATIC
	src/gsoc_labyrinth.cpp	
)

target_include_directories(gsoc_labyrinth
	PUBLIC
		${PROJECT_SOURCE_DIR}/include	
)

link_directories()
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/results/lib)
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/results/bin)

add_executable(gsoc_main
	src/main.cpp	
)

target_link_libraries(gsoc_main
	PRIVATE
		gsoc_labyrinth
)
