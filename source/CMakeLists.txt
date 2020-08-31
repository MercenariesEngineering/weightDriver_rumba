cmake_minimum_required(VERSION 3.10)
project( weightDriver_plugin )

file( GLOB_RECURSE WEIGHTDRIVER_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" )
file( GLOB_RECURSE WEIGHTDRIVER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" )

find_package(rumba REQUIRED)
include_directories(${rumba_INCLUDE_DIRS})
link_directories(${rumba_LIBRARY_DIRS})

add_library( weightDriver_plugin SHARED ${WEIGHTDRIVER_HEADERS} ${WEIGHTDRIVER_SOURCES} )
target_link_libraries( weightDriver_plugin PUBLIC ${rumba_LIBRARIES} )
target_include_directories( weightDriver_plugin PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" )

# On visual, use the release dll runtime (MD) in Debug
if (MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "/MD /Zi /Ob0 /Od /RTC1")
endif()

# Optional output directory
foreach(BUILD_TYPE RELEASE DEBUG DEV CI PROFILE MINSIZEREL RELWITHDEBINFO)
	if(DEFINED rumba_RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE})
		set_target_properties(weightDriver_plugin PROPERTIES
			RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE} "${rumba_RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE}}"
			ARCHIVE_OUTPUT_DIRECTORY_${BUILD_TYPE} "${rumba_RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE}}"
			LIBRARY_OUTPUT_DIRECTORY_${BUILD_TYPE} "${rumba_RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE}}"
		)
	endif()
endforeach()
if(DEFINED rumba_RUNTIME_OUTPUT_DIRECTORY)
	set_target_properties(weightDriver_plugin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${rumba_RUNTIME_OUTPUT_DIRECTORY}")
endif()
