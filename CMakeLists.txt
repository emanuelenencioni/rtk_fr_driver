project(mixer_interface)
cmake_minimum_required(VERSION 2.6)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)
include(GNUInstallDirs)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}") 

include_directories ("${PROJECT_SOURCE_DIR}/include")

add_subdirectory (lib)

include_directories ("${PROJECT_SOURCE_DIR}/lib/Com")

include_directories ("${PROJECT_SOURCE_DIR}/lib/time")

include_directories ("${PROJECT_SOURCE_DIR}/lib/thread_functions")

set(include "${PROJECT_NAME}/include")

set(header ${PROJECT_SOURCE_DIR}/include/ext_dep.h
           ${PROJECT_SOURCE_DIR}/include/typeDef.h
           )

install(FILES ${header} DESTINATION "${include}")

install(
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION ${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}-targets.cmake # Not sure if this is still needed
  )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	VERSION 1.0.0
	COMPATIBILITY AnyNewerVersion
	)

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}Config.cmake.in" 
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${PROJECT_NAME}
	)

#Install the config, configversion and custom find modules
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	DESTINATION "${PROJECT_NAME}"	
	)

add_executable (ComDemo main.cpp)

target_link_libraries (ComDemo Com
                               thread_functions
                               internalTime
                               Threads::Threads
                                )
                                

