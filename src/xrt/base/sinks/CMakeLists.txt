# Copyright 2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_library(
	base_sinks STATIC
	b_frame_source.c
	b_frame_source.h
	b_sink_bridge.c
	b_sink_bridge.h
	b_timing_source.c
	b_timing_source.h
	)
target_include_directories(base_sinks PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
	base_sinks
	PRIVATE aux_util aux_os
	PUBLIC xrt-interfaces
	)
