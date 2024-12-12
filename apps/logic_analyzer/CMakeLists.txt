add_executable(logic_analyzer
	logic_analyzer.c
)

target_compile_options(logic_analyzer PRIVATE -Wall)

target_link_libraries(logic_analyzer
	pico_stdlib
	pico_util
	hardware_pio
	hardware_dma
	libpicohsdaoh
)
pico_generate_pio_header(logic_analyzer ${CMAKE_CURRENT_LIST_DIR}/16bit_input.pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(logic_analyzer 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(logic_analyzer)
