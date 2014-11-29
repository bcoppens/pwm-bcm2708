include_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable (ws2812_bcm2708_demo ws2812_bcm2708_demo.c)
target_link_libraries (ws2812_bcm2708_demo ws2812_bcm2708)
