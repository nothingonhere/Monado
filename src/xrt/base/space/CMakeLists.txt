# Copyright 2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_library(
	base_space STATIC b_space_overseer.c b_space_overseer.h b_tracked_device.c
			  b_tracked_device.h
	)
target_include_directories(base_space PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
	base_space
	PRIVATE aux_util aux_os aux_math
	PUBLIC xrt-interfaces
	)
