include_directories(${CMAKE_SOURCE_DIR}/lib)

list(APPEND CMAKE_C_FLAGS "-std=gnu11")
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

add_executable (ws2812_bcm2708_demo ws2812_bcm2708_demo.c)
add_executable (clock clock.cc)
add_executable (stress stress.c)
target_link_libraries (ws2812_bcm2708_demo ws2812_bcm2708)
target_link_libraries (stress ws2812_bcm2708)

# TODO urgh
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")
target_link_libraries (clock ws2812_bcm2708)
