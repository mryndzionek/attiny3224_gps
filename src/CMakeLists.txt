include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB PFF_SRCS
     "../external/pff3a/*.c"
)

include_directories("../external/pff3a/")

avr_add_library(
    pff
    ${PFF_SRCS}
)

file(GLOB ST7789_SRCS
     "../external/st7789/src/*.c"
     "driver_st7789_interface.c"
     "display_driver.c"
)

include_directories("../external/st7789/src/"
                    "../external/st7789/interface/")

avr_add_library(
    st7789
    ${ST7789_SRCS}
)

target_compile_definitions(st7789 PUBLIC ST7789_BUFFER_SIZE=512)

avr_add_executable(
    map_test
    map_test.c
    diskio.c
)

avr_target_link_libraries(map_test pff)
avr_target_link_libraries(map_test st7789)

