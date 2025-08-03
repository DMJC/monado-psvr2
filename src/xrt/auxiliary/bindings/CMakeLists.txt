# Copyright 2019-2025, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

# Binding generation: pass filename to generate
function(bindings_gen template output)
	add_custom_command(
		OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${output}"
		COMMAND
			${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/bindings.py
			${CMAKE_CURRENT_SOURCE_DIR}/bindings.json
			"${CMAKE_CURRENT_SOURCE_DIR}/${template}"
			"${CMAKE_CURRENT_BINARY_DIR}/${output}"
		VERBATIM
		DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/bindings.py
			${CMAKE_CURRENT_SOURCE_DIR}/bindings.json
		COMMENT "Generating ${output}"
		)
endfunction()

bindings_gen(b_generated_bindings_helpers.c.template b_generated_bindings_helpers.c)

# Bindings library.
add_library(
	aux_generated_bindings_helpers STATIC
	${CMAKE_CURRENT_BINARY_DIR}/b_generated_bindings_helpers.c
	)

# needed globally for steamvr input profile generation in steamvr target
set_property(GLOBAL PROPERTY AUX_BINDINGS_DIR_PROP "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(aux_generated_bindings_helpers PRIVATE xrt-interfaces)
target_include_directories(aux_generated_bindings_helpers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

set(AUX_BINDINGS_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}"
    CACHE PATH "Path to the shared bindings generation code"
	)
