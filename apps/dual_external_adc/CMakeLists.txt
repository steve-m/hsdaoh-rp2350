add_executable(dual_external_adc
	dual_external_adc.c
)

target_compile_options(dual_external_adc PRIVATE -Wall)

target_link_libraries(dual_external_adc
	pico_stdlib
	pico_util
	hardware_pio
	hardware_dma
	libpicohsdaoh
)
pico_generate_pio_header(dual_external_adc ${CMAKE_CURRENT_LIST_DIR}/adc_24bit_input.pio)
pico_generate_pio_header(dual_external_adc ${CMAKE_CURRENT_LIST_DIR}/pcm1802_fmt00.pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(dual_external_adc 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(dual_external_adc)
