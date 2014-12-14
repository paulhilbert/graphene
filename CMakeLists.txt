cmake_minimum_required(VERSION 2.8.8)
project(graphene)

set (graphene_VERSION_MAJOR 1)
set (graphene_VERSION_MINOR 0)

configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_SOURCE_DIR}/include/config.h"
)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("/usr/include/eigen3")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-fPIC)
endif()

# c++11
if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-std=c++1y)
endif()

# boost
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost COMPONENTS regex filesystem program_options system) 

# opengl
find_package(OpenGL)
find_package(GLEW)
find_package(Harmont)


file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "FW/*.cpp" "GUI/*.cpp")
file (GLOB_RECURSE lib RELATIVE "${PROJECT_SOURCE_DIR}" "Library/Rendered/*.cpp")
file (GLOB_RECURSE qt5 RELATIVE "${PROJECT_SOURCE_DIR}" "Backends/Qt5/*.cpp")
if(Boost_FOUND AND OPENGL_FOUND AND GLEW_FOUND AND HARMONT_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
	include_directories(${OPENGL_INCLUDE_DIR})
	include_directories(${GLEW_INCLUDE_DIRS})
	include_directories(${HARMONT_INCLUDE_DIRS})
	#add_library(Graphene SHARED ${obj} ${lib})
	add_library(Graphene SHARED ${obj})
	target_link_libraries(Graphene ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${HARMONT_LIBRARIES} "dl")
	add_executable(graphene "graphene.cpp")
	target_link_libraries(graphene Graphene)

	# qt backend
	set(CMAKE_AUTOMOC ON)
	set(CMAKE_INCLUDE_CURRENT_DIR ON)
	find_package(Qt5Widgets REQUIRED)

	include_directories("${PROJECT_SOURCE_DIR}Backends/Qt5")
	add_library(GrapheneQt5 SHARED ${qt5})
	target_link_libraries(GrapheneQt5 Graphene)
	qt5_use_modules(GrapheneQt5 Widgets OpenGL)
endif()

# binary
install (TARGETS graphene DESTINATION bin)
install (TARGETS Graphene GrapheneQt5 DESTINATION lib)
install (FILES "bin/createBase" "bin/createVisualizer" DESTINATION bin)

# header
install (DIRECTORY FW DESTINATION include/graphene FILES_MATCHING PATTERN "*.h" PATTERN "*.inl")
install (DIRECTORY GUI DESTINATION include/graphene FILES_MATCHING PATTERN "*.h" PATTERN "*.inl")
install (DIRECTORY include DESTINATION include/graphene FILES_MATCHING PATTERN "*.h" PATTERN "*.inl")
install (DIRECTORY Library DESTINATION include/graphene FILES_MATCHING PATTERN "*.h" PATTERN "*.inl" PATTERN "*.frag" PATTERN "*.vert" PATTERN "*.geom" PATTERN "*Skeleton.*" PATTERN "SConstruct" PATTERN "*.hdr")

# icons
file (GLOB icons RELATIVE "${PROJECT_SOURCE_DIR}" "Icons/*.png")
install (FILES ${icons} DESTINATION share/icons/graphene)
