# Copyright 2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_library(
	base_devices STATIC
	b_body_tracker.cpp
	b_body_tracker.h
	b_body_tracker.hpp
	b_hand_tracker.cpp
	b_hand_tracker.h
	b_hand_tracker.hpp
	b_io_cache.cpp
	b_io_cache.h
	b_io_cache.hpp
	b_system_devices.c
	b_system_devices.h
	)
target_include_directories(base_devices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
	base_devices
	PRIVATE aux_util aux_math
	PUBLIC xrt-interfaces
	)
