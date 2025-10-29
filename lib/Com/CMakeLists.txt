cmake_minimum_required(VERSION 2.6)

set(com "${PROJECT_NAME}/lib/Com")

include_directories("${PROJECT_SOURCE_DIR}/include") 

include_directories("${PROJECT_SOURCE_DIR}/lib/time") 

add_library(Com Com.cpp)

add_library(${PROJECT_NAME}::Com ALIAS Com)

install(FILES ${PROJECT_SOURCE_DIR}/lib/Com/Com.h DESTINATION "${com}")

include(GNUInstallDirs)
install(TARGETS Com
        EXPORT ${PROJECT_NAME}-targets
        ARCHIVE DESTINATION ${com}        
        LIBRARY DESTINATION ${com}
	PUBLIC_HEADER DESTINATION ${com}  
        INCLUDES DESTINATION ${com} 	      
       )
       
