add_executable(sdr
	sdr.c
)

target_compile_options(sdr PRIVATE -Wall)

target_link_libraries(sdr
	pico_stdlib
	pico_util
	hardware_pio
	hardware_dma
	hardware_i2c
	libpicohsdaoh
)
pico_generate_pio_header(sdr ${CMAKE_CURRENT_LIST_DIR}/adc_20bit_input.pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(sdr 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(sdr)
