# Copyright 2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_subdirectory(app)
add_subdirectory(devices)
add_subdirectory(sinks)
add_subdirectory(space)

add_library(base INTERFACE)
target_include_directories(base INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_sources(base INTERFACE b_documentation.h)
target_link_libraries(base INTERFACE base_app base_devices base_sinks base_space)
