// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Frame source interface, frame sinks can be added and removed at runtime.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_frame.h"
#include "xrt/xrt_results.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * @interface xrt_frame_source
 *
 * Something that produces frames, and pushes them to all sinks added to it.
 * The frame counterpart of @ref t_timing_event_source.
 *
 * Implementations **must** also implement @ref xrt_frame_node. Once
 * @ref xrt_frame_source::remove_sink returns the sink will not be pushed to
 * again, so it is safe to destroy it.
 *
 * The same frame is pushed to all sinks, so they must not modify it. Sinks
 * must not add or remove sinks on the same source from within
 * @ref xrt_frame_sink::push_frame.
 *
 * @ingroup xrt_iface
 */
struct xrt_frame_source
{
	/*!
	 * Add a sink that frames will be pushed to.
	 *
	 * @param xfsrc The frame source.
	 * @param xs    The sink to add.
	 *
	 * @return XRT_SUCCESS, XRT_ERROR_ALLOCATION if the source is full, or
	 *         another error if the source can't take sinks right now.
	 */
	xrt_result_t (*add_sink)(struct xrt_frame_source *xfsrc, struct xrt_frame_sink *xs);

	/*!
	 * Remove a sink, does nothing if it hasn't been added. Waits for any
	 * in-flight push to the sink to return.
	 *
	 * @param xfsrc The frame source.
	 * @param xs    The sink to remove.
	 */
	void (*remove_sink)(struct xrt_frame_source *xfsrc, struct xrt_frame_sink *xs);
};

/*!
 * @copydoc xrt_frame_source::add_sink
 *
 * Helper for calling through the function pointer.
 *
 * @public @memberof xrt_frame_source
 */
XRT_NONNULL_ALL static inline xrt_result_t
xrt_frame_source_add_sink(struct xrt_frame_source *xfsrc, struct xrt_frame_sink *xs)
{
	return xfsrc->add_sink(xfsrc, xs);
}

/*!
 * @copydoc xrt_frame_source::remove_sink
 *
 * Helper for calling through the function pointer.
 *
 * @public @memberof xrt_frame_source
 */
XRT_NONNULL_ALL static inline void
xrt_frame_source_remove_sink(struct xrt_frame_source *xfsrc, struct xrt_frame_sink *xs)
{
	xfsrc->remove_sink(xfsrc, xs);
}


#ifdef __cplusplus
}
#endif
