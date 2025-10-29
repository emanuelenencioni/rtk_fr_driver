cmake_minimum_required(VERSION 2.6)

set(time "${PROJECT_NAME}/lib/time")

include_directories("${PROJECT_SOURCE_DIR}/include") 

add_library(internalTime internalTime.cpp)

add_library(${PROJECT_NAME}::internalTime ALIAS internalTime)

set_property(TARGET internalTime PROPERTY CXX_STANDARD 11)

install(FILES ${PROJECT_SOURCE_DIR}/lib/time/internalTime.h DESTINATION "${time}")

include(GNUInstallDirs)
install(TARGETS internalTime
        EXPORT ${PROJECT_NAME}-targets
        ARCHIVE DESTINATION ${time}
        LIBRARY DESTINATION ${time}
	PUBLIC_HEADER DESTINATION ${time}  
        INCLUDES DESTINATION ${time}  	                     
       )
       

