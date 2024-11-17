add_executable(external_adc
	external_adc.c
)

target_compile_options(external_adc PRIVATE -Wall)

target_link_libraries(external_adc
	pico_stdlib
	pico_util
	hardware_pio
	hardware_dma
	libpicohsdaoh
)
pico_generate_pio_header(external_adc ${CMAKE_CURRENT_LIST_DIR}/adc_12bit_input.pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(external_adc 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(external_adc)
