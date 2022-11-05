find_package(Boost COMPONENTS program_options)

set(EXAMPLES ntrip_client)


foreach(example IN ITEMS ${EXAMPLES})
    add_executable(${example}_example ${example}_example.cpp)
    target_link_libraries(${example}_example PRIVATE Boost::program_options lunar::ntrip_client spdlog::spdlog)
endforeach()
