add_executable(internal_adc
	internal_adc.c
)

target_compile_options(internal_adc PRIVATE -Wall)

target_link_libraries(internal_adc
	pico_stdlib
	hardware_adc
	hardware_dma
	pico_util
	libpicohsdaoh
)

# enable usb output, disable uart output
pico_enable_stdio_usb(internal_adc 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(internal_adc)
