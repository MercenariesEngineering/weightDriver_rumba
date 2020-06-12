cmake_minimum_required(VERSION 3.10)
project( weightDriver )

file( GLOB_RECURSE WEIGHTDRIVER_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" )
file( GLOB_RECURSE WEIGHTDRIVER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" )

include_directories( ${RUMBA_SDK}/include )
link_directories( ${RUMBA_SDK}/lib )
 
add_library( weightDriver SHARED ${WEIGHTDRIVER_HEADERS} ${WEIGHTDRIVER_SOURCES} ) 
target_link_libraries( weightDriver PUBLIC kernel Iex Imath )
target_include_directories( weightDriver PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" )

# On visual, use the release dll runtime (MD) in Debug
if (MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "/MD /Zi /Ob0 /Od /RTC1")
endif()
