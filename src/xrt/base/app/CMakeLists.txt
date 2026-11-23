# Copyright 2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_library(
	base_app STATIC
	b_app_policy.c
	b_app_policy.h
	b_session.c
	b_session.h
	b_system.c
	b_system.h
	)
target_include_directories(base_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
	base_app
	PRIVATE aux_util aux_os aux_math
	PUBLIC xrt-interfaces
	)
