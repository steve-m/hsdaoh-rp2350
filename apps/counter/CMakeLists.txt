add_executable(counter
	counter.c
)

target_compile_options(counter PRIVATE -Wall)

target_link_libraries(counter
	pico_stdlib
	pico_multicore
	pico_util
	hardware_pio
	hardware_dma
	libpicohsdaoh
)
pico_generate_pio_header(counter ${CMAKE_CURRENT_LIST_DIR}/counter.pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(counter 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(counter)
