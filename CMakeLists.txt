cmake_minimum_required(VERSION 3.5)

project(ntrip)

set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)

option(NTRIP_BUILD_EXAMPLES "Build ntrip examples" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system)


set(src_MAIN
  #src/ntrip_caster.cc
  src/ntrip_client.cc
  #src/ntrip_server.cc
  src/ntrip_util.cc
  include/ntrip/mount_point.h
  include/ntrip/ntrip_caster.h
  include/ntrip/ntrip_server.h
  include/ntrip/ntrip_client.h
  include/ntrip/ntrip_util.h
)

add_library(${PROJECT_NAME} ${src_MAIN})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PRIVATE Boost::boost Boost::disable_autolinking)


set_target_properties(${PROJECT_NAME} PROPERTIES
  DEBUG_POSTFIX _d
  RELWITHDEBINFO_POSTFIX _rd
)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-config
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT ${PROJECT_NAME}-config
  DESTINATION lib/cmake/${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
)

install(DIRECTORY include/ntrip DESTINATION include)

if (NTRIP_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif (NTRIP_BUILD_EXAMPLES)
