// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Base frame source.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup base
 */

#pragma once

#include "xrt/xrt_frame.h"
#include "xrt/xrt_frame_source.h"
#include "xrt/xrt_results.h"


#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Creates a @ref xrt_frame_source, frames pushed to @p out_sink are pushed to
 * all sinks added to @p out_source.
 *
 * @ingroup base
 */
xrt_result_t
b_frame_source_create(struct xrt_frame_context *xfctx,
                      struct xrt_frame_sink **out_sink,
                      struct xrt_frame_source **out_source);

#ifdef __cplusplus
}
#endif
