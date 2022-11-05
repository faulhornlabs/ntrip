cmake_minimum_required(VERSION 3.10)
project(lunar_ntrip_client)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)



find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)


set(PUB_HDRS
    include/lunar/ntrip_client.h
    include/lunar/ntrip_util.h
    include/lunar/bittools.hpp
    include/lunar/crc.h
    include/lunar/rtcm_parser.h
)

set(SRCS
    src/ntrip_client.cpp
    src/ntrip_util.cpp
    src/rtcm_parser.cpp

)

add_library(lunar_ntrip_client ${PUB_HDRS} ${SRCS})
add_library(lunar::ntrip_client ALIAS lunar_ntrip_client)
target_include_directories(lunar_ntrip_client
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/lunar_ntrip_client>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(lunar_ntrip_client
    PUBLIC
        Boost::boost
    PRIVATE
        spdlog::spdlog
)

set_target_properties(lunar_ntrip_client PROPERTIES
    DEBUG_POSTFIX _d
    RELWITHDEBINFO_POSTFIX _rd
    POSITION_INDEPENDENT_CODE ON
    FOLDER libs
)


add_subdirectory(examples)

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
