cmake_minimum_required (VERSION 2.6)

# Maps to a solution file (ETM.sln).
project (ETM)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Set compiler flags and options. 
if( MSVC )
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
elseif( UNIX OR MINGW )
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O3 -fomit-frame-pointer -Wall -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-attributes -Werror -Wno-strict-overflow -Wno-unknown-pragmas -Wno-stringop-overflow -std=c99")
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pointer-sign")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -fomit-frame-pointer -Wall -Werror -std=c++11")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-attributes -Wno-strict-overflow -Wno-sign-compare -Wno-unused-value -Wno-unknown-pragmas")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lm -lstdc++")
endif()

# Command to output information to the console
message ("c Flags: " ${CMAKE_C_FLAGS})
message ("cxx Flags: " ${CMAKE_CXX_FLAGS})
message ("linker Flags: " ${CMAKE_EXE_LINKER_FLAGS})

# Sub-directories where more CMakeLists.txt exist
add_subdirectory(src)
add_subdirectory(app)
