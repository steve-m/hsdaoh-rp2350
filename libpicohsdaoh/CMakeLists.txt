# Note we are using INTERFACE so that the library can be configured per-app
# with compile-time defines

add_library(libpicohsdaoh INTERFACE)

target_sources(libpicohsdaoh INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/picohsdaoh.c
	${CMAKE_CURRENT_LIST_DIR}/picohsdaoh.h
	${CMAKE_CURRENT_LIST_DIR}/data_packet.c
	${CMAKE_CURRENT_LIST_DIR}/data_packet.h
	)

target_include_directories(libpicohsdaoh INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(libpicohsdaoh INTERFACE
	pico_base_headers
	pico_util
	pico_multicore
	hardware_dma
	)
