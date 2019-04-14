#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)


SET (projectname "RealSenseDNNExample")
project(${projectname})

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# elseif(COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# endif()



find_package(OpenCV REQUIRED)

find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)


set(DEPENDENCIES realsense2 ${OpenCV_LIBS}
                 /home/jessy/librealsense/wrappers/librealsense2.so
                 /usr/lib/x86_64-linux-gnu/libglfw.so
                /usr/lib/x86_64-linux-gnu/libglut.so 
                /usr/lib/x86_64-linux-gnu/libGL.so 
                /usr/lib/x86_64-linux-gnu/libGLU.so 
                /usr/lib/x86_64-linux-gnu/libGLEW.so )

add_executable(${projectname}            
               Demo.cpp
               cv-helpers.hpp
               include/actionmodule.cpp)
target_link_libraries(${projectname} ${DEPENDENCIES}
                        Qt5::Core 
                        Qt5::Network
                        )
include_directories(include
                    examples
                    include/third-party
                    include/third-party/glfw
                    include/third-party/imgui
                    include/librealsense2
                    include/common
                    ${CMAKE_CURRENT_LIST_DIR}/include
				    ${Qt5Core_INCLUDE_DIRS}
				    ${Qt5Network_INCLUDE_DIRS})

add_definitions(${Qt5Core_DEFINITIONS}
				${Qt5Network_DEFINITIONS})


set_target_properties (${projectname} PROPERTIES
    FOLDER "Examples/OpenCV"
)