cmake_minimum_required(VERSION 3.10)
project(EV_ChargeRoute CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

file(GLOB EV_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)
add_executable(ev_chargeroute ${EV_SOURCES})
target_include_directories(ev_chargeroute PRIVATE ${CMAKE_SOURCE_DIR}/include)

# The 2D simulation needs SFML 3.x (this version of Simulation.cpp uses the SFML 3 API).
find_package(SFML 3 COMPONENTS Graphics Window System QUIET)
if(SFML_FOUND)
  target_link_libraries(ev_chargeroute PRIVATE SFML::Graphics SFML::Window SFML::System)
else()
  message(WARNING "SFML 3.x not found - building WITHOUT the window. "
                  "Use --cli and --test, or install SFML 3 and re-run cmake (see README).")
  target_compile_definitions(ev_chargeroute PRIVATE EV_NO_GUI)
endif()

if(MSVC)
  target_compile_options(ev_chargeroute PRIVATE /W3)
else()
  target_compile_options(ev_chargeroute PRIVATE -Wall -Wextra)
endif()

# copy dataset + assets next to the executable so it can be started from anywhere
add_custom_command(TARGET ev_chargeroute POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/dataset $<TARGET_FILE_DIR:ev_chargeroute>/dataset
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/assets  $<TARGET_FILE_DIR:ev_chargeroute>/assets)

enable_testing()
add_test(NAME five_scenarios COMMAND ev_chargeroute --test)
