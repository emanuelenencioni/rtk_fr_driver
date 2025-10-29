cmake_minimum_required(VERSION 2.6)

include_directories ("${PROJECT_SOURCE_DIR}/lib/Com")
add_subdirectory (Com) 

include_directories ("${PROJECT_SOURCE_DIR}/lib/time")
add_subdirectory (time)

include_directories ("${PROJECT_SOURCE_DIR}/lib/thread_functions")
add_subdirectory (thread_functions)





