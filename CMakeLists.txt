cmake_minimum_required(VERSION 3.5)
project(basic_agent_ST)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# If you are on mac x86
#set(CMAKE_OSX_ARCHITECTURES x86_64)

include(lib/basic.cmake)

include_directories(
    matlab/cpp_generated
    lib
    log
)

link_directories(
    lib
)

# Shared setting for all agents
add_library(agent_common STATIC
    log/screen_print.c
    log/logvars.cc
    log/additional_logging.cpp
    matlab/cpp_generated/primitive.h
    matlab/cpp_generated/rt_nonfinite.h
)
target_link_libraries(agent_common PUBLIC agent_lib ${lib_ws32})
target_compile_options(agent_common PUBLIC -DNOT_INLINED -DSERVER)
set_target_properties(agent_common PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${AGENT_FOLDER}/lib)

# Basic agent
add_executable(basic_agent preliminaryWork/starting_point.cpp)
target_link_libraries(basic_agent agent_common)

# Pass Primitive agent
add_executable(pass_primitive_agent preliminaryWork/pass_primitive_agent.cpp)
target_link_libraries(pass_primitive_agent agent_common)

set_target_properties(basic_agent pass_primitive_agent
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${AGENT_FOLDER})

