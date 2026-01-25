add_executable(external_dualchan_adc
	external_dualchan_adc.c
)

target_compile_options(external_dualchan_adc PRIVATE -Wall)

target_link_libraries(external_dualchan_adc
	pico_stdlib
	pico_util
	hardware_pio
	hardware_dma
	libpicohsdaoh
)
pico_generate_pio_header(external_dualchan_adc ${CMAKE_CURRENT_LIST_DIR}/dualchan_adc_12bit_input.pio)
pico_generate_pio_header(external_dualchan_adc ${CMAKE_CURRENT_LIST_DIR}/../external_adc/pcm1802_fmt00.pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(external_dualchan_adc 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(external_dualchan_adc)
