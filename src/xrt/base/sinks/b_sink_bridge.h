// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Moves frames and timing events between two frame contexts.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup base
 */

#pragma once

#include "xrt/xrt_frame.h"
#include "xrt/xrt_frame_source.h"
#include "xrt/xrt_results.h"
#include "xrt/xrt_tracking.h"

#include "tracking/t_time_sync.h"


#ifdef __cplusplus
extern "C" {
#endif

//! Number of frame sources.
#define B_SINK_BRIDGE_MAX_FRAME_SOURCES XRT_TRACKING_MAX_FRAME_SOURCES

/*!
 * Number of timing event sources. Synced cameras usually share one, but each
 * frame source could have its own.
 */
#define B_SINK_BRIDGE_MAX_TIMING_SOURCES XRT_TRACKING_MAX_FRAME_SOURCES

//! How many sinks can be added to each receiving source.
#define B_SINK_BRIDGE_MAX_SINKS 4

/*!
 * Lets a long lived producer, like a camera, keep running while the frame
 * context that consumes its data is torn down and created again.
 *
 * The bridge has one node in the sending context and one in the receiving
 * context, it is freed once the nodes in all contexts it has been in are
 * destroyed, in any order. The producer pushes into the plain sinks from
 * @ref b_sink_bridge_get_frame_sink and @ref b_sink_bridge_get_timing_sink, so
 * it doesn't need to know about the bridge. Frame and timing event sources are
 * indexed separately, which index the producer uses for what is up to it and
 * the consumers.
 *
 * The receiving context can keep references to frames, in queues for example,
 * for longer than the sending context lives. If the frames' memory belongs to
 * the producer, like a driver's frame pool, the receiving context must be
 * destroyed first.
 *
 * The receiving side is one context at a time. After
 * @ref b_sink_bridge_attach consumers add their sinks to the sources from
 * @ref b_sink_bridge_get_frame_source and @ref b_sink_bridge_get_timing_source.
 * When the receiving context is broken apart all of its sinks are dropped, so
 * consumers living in it don't need to remove them, and must add them again
 * after the next attach. Sinks destroyed before the receiving context, like a
 * controller that is unplugged, must be removed.
 *
 * @ingroup base
 */
struct b_sink_bridge;

/*!
 * Create a bridge, it is added to @p send_xfctx.
 *
 * @ingroup base
 */
xrt_result_t
b_sink_bridge_create(struct xrt_frame_context *send_xfctx, struct b_sink_bridge **out_bsb);

/*!
 * Attach the receiving side to @p recv_xfctx. Fails if it is already attached
 * to a context that hasn't been destroyed yet.
 *
 * @public @memberof b_sink_bridge
 */
xrt_result_t
b_sink_bridge_attach(struct b_sink_bridge *bsb, struct xrt_frame_context *recv_xfctx);

/*!
 * Sink for the producer to push frames for frame source @p index into.
 *
 * @public @memberof b_sink_bridge
 */
struct xrt_frame_sink *
b_sink_bridge_get_frame_sink(struct b_sink_bridge *bsb, uint32_t index);

/*!
 * Sink for the producer to push timing events for timing event source
 * @p index into.
 *
 * @public @memberof b_sink_bridge
 */
struct t_timing_event_sink *
b_sink_bridge_get_timing_sink(struct b_sink_bridge *bsb, uint32_t index);

/*!
 * Receiving side frame source @p index, adding sinks to it fails when the
 * bridge isn't attached.
 *
 * @public @memberof b_sink_bridge
 */
struct xrt_frame_source *
b_sink_bridge_get_frame_source(struct b_sink_bridge *bsb, uint32_t index);

/*!
 * Receiving side timing event source @p index, adding sinks to it fails when
 * the bridge isn't attached.
 *
 * @public @memberof b_sink_bridge
 */
struct t_timing_event_source *
b_sink_bridge_get_timing_source(struct b_sink_bridge *bsb, uint32_t index);

#ifdef __cplusplus
}
#endif
