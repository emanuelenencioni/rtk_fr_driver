project(ComDemo)
cmake_minimum_required(VERSION 2.6)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(mixer_interface REQUIRED)

include_directories (${MIXER_INTERFACE_INCLUDE_DIRS})

add_executable (Com_test main.cpp)

target_link_libraries (Com_test mixer_interface::Com
                                mixer_interface::thread_functions
                                mixer_interface::internalTime
                                Threads::Threads
                                )



