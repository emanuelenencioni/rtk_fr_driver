cmake_minimum_required(VERSION 2.6)

set(thread_functions "${PROJECT_NAME}/lib/thread_functions")

include_directories("${PROJECT_SOURCE_DIR}") 

include_directories("${PROJECT_SOURCE_DIR}/include") 

add_library(thread_functions thread_functions.cpp)

add_library(${PROJECT_NAME}::thread_functions ALIAS thread_functions)

set_property(TARGET thread_functions PROPERTY CXX_STANDARD 11)

install(FILES ${PROJECT_SOURCE_DIR}/lib/thread_functions/thread_functions.h DESTINATION "${thread_functions}")

include(GNUInstallDirs)
install(TARGETS thread_functions
        EXPORT ${PROJECT_NAME}-targets
        ARCHIVE DESTINATION ${thread_functions}
        LIBRARY DESTINATION ${thread_functions}
	PUBLIC_HEADER DESTINATION ${thread_functions}  
        INCLUDES DESTINATION ${thread_functions}                
       )


