// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Moves frames and timing events between two frame contexts.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup base
 */

#include "util/u_misc.h"

#include "os/os_threading.h"

#include "b_sink_bridge.h"

#include <assert.h>


struct b_sink_bridge_frame
{
	//! Sending side.
	struct xrt_frame_sink sink;

	//! Receiving side.
	struct xrt_frame_source source;

	struct xrt_frame_sink *sinks[B_SINK_BRIDGE_MAX_SINKS];
};

struct b_sink_bridge_timing
{
	//! Sending side.
	struct t_timing_event_sink sink;

	//! Receiving side.
	struct t_timing_event_source source;

	struct t_timing_event_sink *sinks[B_SINK_BRIDGE_MAX_SINKS];
};

struct b_sink_bridge
{
	//! One for the send node, plus one while the receive node is in a context.
	struct xrt_reference ref;

	struct xrt_frame_node send_node;
	struct xrt_frame_node recv_node;

	/*!
	 * Read locked while pushing, sinks are not reference counted so this is
	 * what makes sure nothing is pushed to them after remove_sink has
	 * returned or the receiving context has been broken apart.
	 */
	struct os_rwlock lock;

	//! Cleared when the send node is broken apart.
	bool sending;

	//! Set on attach, cleared when the receive node is broken apart.
	bool receiving;

	//! Set on attach, cleared when the receive node is destroyed.
	bool recv_in_context;

	struct b_sink_bridge_frame frames[B_SINK_BRIDGE_MAX_FRAME_SOURCES];
	struct b_sink_bridge_timing timings[B_SINK_BRIDGE_MAX_TIMING_SOURCES];
};


/*
 *
 * Helpers.
 *
 */

/*
 * Using frames[IDX].sink in container_of directly needs offsetof to accept an
 * array index, which wasn't made explicit until C17 (DR 496) and GCC on Android
 * rejects. So step back to element 0 first, drop this once we are on C17.
 */
#define BRIDGE_FROM(PTR, TYPE, FIELD, ARRAY, IDX)                                                                      \
	container_of((container_of(PTR, TYPE, FIELD) - (IDX)), struct b_sink_bridge, ARRAY)

static void
bridge_unref(struct b_sink_bridge *bsb)
{
	if (!xrt_reference_dec_and_is_zero(&bsb->ref)) {
		return;
	}

	os_rwlock_destroy(&bsb->lock);
	free(bsb);
}

static void
push_frame(struct b_sink_bridge *bsb, uint32_t index, struct xrt_frame *xf)
{
	struct b_sink_bridge_frame *f = &bsb->frames[index];

	os_rwlock_read_lock(&bsb->lock);
	if (bsb->sending && bsb->receiving) {
		for (size_t i = 0; i < ARRAY_SIZE(f->sinks); i++) {
			if (f->sinks[i] != NULL) {
				xrt_sink_push_frame(f->sinks[i], xf);
			}
		}
	}
	os_rwlock_unlock(&bsb->lock);
}

static void
push_timing_event(struct b_sink_bridge *bsb, uint32_t index, const struct t_timing_event *event)
{
	struct b_sink_bridge_timing *t = &bsb->timings[index];

	os_rwlock_read_lock(&bsb->lock);
	if (bsb->sending && bsb->receiving) {
		for (size_t i = 0; i < ARRAY_SIZE(t->sinks); i++) {
			if (t->sinks[i] != NULL) {
				t_timing_event_sink_push_timing_event(t->sinks[i], event);
			}
		}
	}
	os_rwlock_unlock(&bsb->lock);
}

static xrt_result_t
frame_source_add_sink(struct b_sink_bridge *bsb, uint32_t index, struct xrt_frame_sink *xs)
{
	struct b_sink_bridge_frame *f = &bsb->frames[index];
	xrt_result_t xret = XRT_ERROR_ALLOCATION;

	os_rwlock_write_lock(&bsb->lock);
	if (!bsb->receiving) {
		xret = XRT_ERROR_INVALID_ARGUMENT;
	}
	for (size_t i = 0; bsb->receiving && i < ARRAY_SIZE(f->sinks); i++) {
		if (f->sinks[i] == NULL) {
			f->sinks[i] = xs;
			xret = XRT_SUCCESS;
			break;
		}
	}
	os_rwlock_unlock(&bsb->lock);

	return xret;
}

static void
frame_source_remove_sink(struct b_sink_bridge *bsb, uint32_t index, struct xrt_frame_sink *xs)
{
	struct b_sink_bridge_frame *f = &bsb->frames[index];

	os_rwlock_write_lock(&bsb->lock);
	for (size_t i = 0; i < ARRAY_SIZE(f->sinks); i++) {
		if (f->sinks[i] == xs) {
			f->sinks[i] = NULL;
		}
	}
	os_rwlock_unlock(&bsb->lock);
}

static int
timing_source_add_sink(struct b_sink_bridge *bsb, uint32_t index, struct t_timing_event_sink *sink)
{
	struct b_sink_bridge_timing *t = &bsb->timings[index];
	int ret = -1;

	os_rwlock_write_lock(&bsb->lock);
	for (size_t i = 0; bsb->receiving && i < ARRAY_SIZE(t->sinks); i++) {
		if (t->sinks[i] == NULL) {
			t->sinks[i] = sink;
			ret = 0;
			break;
		}
	}
	os_rwlock_unlock(&bsb->lock);

	return ret;
}

static void
timing_source_remove_sink(struct b_sink_bridge *bsb, uint32_t index, struct t_timing_event_sink *sink)
{
	struct b_sink_bridge_timing *t = &bsb->timings[index];

	os_rwlock_write_lock(&bsb->lock);
	for (size_t i = 0; i < ARRAY_SIZE(t->sinks); i++) {
		if (t->sinks[i] == sink) {
			t->sinks[i] = NULL;
		}
	}
	os_rwlock_unlock(&bsb->lock);
}


/*
 *
 * Per source functions, the index is baked in so no back pointer is needed.
 *
 */

#define FRAME_FROM(PTR, FIELD, IDX) BRIDGE_FROM(PTR, struct b_sink_bridge_frame, FIELD, frames, IDX)
#define TIMING_FROM(PTR, FIELD, IDX) BRIDGE_FROM(PTR, struct b_sink_bridge_timing, FIELD, timings, IDX)

#define DEFINE_FRAME_FUNCS(IDX)                                                                                        \
	static void push_frame_##IDX(struct xrt_frame_sink *xs, struct xrt_frame *xf)                                  \
	{                                                                                                              \
		push_frame(FRAME_FROM(xs, sink, IDX), IDX, xf);                                                        \
	}                                                                                                              \
	static xrt_result_t frame_add_sink_##IDX(struct xrt_frame_source *xfsrc, struct xrt_frame_sink *xs)            \
	{                                                                                                              \
		return frame_source_add_sink(FRAME_FROM(xfsrc, source, IDX), IDX, xs);                                 \
	}                                                                                                              \
	static void frame_remove_sink_##IDX(struct xrt_frame_source *xfsrc, struct xrt_frame_sink *xs)                 \
	{                                                                                                              \
		frame_source_remove_sink(FRAME_FROM(xfsrc, source, IDX), IDX, xs);                                     \
	}

#define DEFINE_TIMING_FUNCS(IDX)                                                                                       \
	static void push_timing_event_##IDX(struct t_timing_event_sink *sink, const struct t_timing_event *event)      \
	{                                                                                                              \
		push_timing_event(TIMING_FROM(sink, sink, IDX), IDX, event);                                           \
	}                                                                                                              \
	static int timing_add_sink_##IDX(struct t_timing_event_source *source, struct t_timing_event_sink *sink)       \
	{                                                                                                              \
		return timing_source_add_sink(TIMING_FROM(source, source, IDX), IDX, sink);                            \
	}                                                                                                              \
	static void timing_remove_sink_##IDX(struct t_timing_event_source *source, struct t_timing_event_sink *sink)   \
	{                                                                                                              \
		timing_source_remove_sink(TIMING_FROM(source, source, IDX), IDX, sink);                                \
	}

XRT_TRACKING_FOR_EACH_FRAME_SOURCE(DEFINE_FRAME_FUNCS)
XRT_TRACKING_FOR_EACH_FRAME_SOURCE(DEFINE_TIMING_FUNCS)
#undef DEFINE_FRAME_FUNCS
#undef DEFINE_TIMING_FUNCS
#undef FRAME_FROM
#undef TIMING_FROM
#undef BRIDGE_FROM

#define SETUP_FRAME(IDX)                                                                                               \
	do {                                                                                                           \
		struct b_sink_bridge_frame *f = &bsb->frames[IDX];                                                     \
		f->sink.push_frame = push_frame_##IDX;                                                                 \
		f->source.add_sink = frame_add_sink_##IDX;                                                             \
		f->source.remove_sink = frame_remove_sink_##IDX;                                                       \
	} while (false);

#define SETUP_TIMING(IDX)                                                                                              \
	do {                                                                                                           \
		struct b_sink_bridge_timing *t = &bsb->timings[IDX];                                                   \
		t->sink.push_timing_event = push_timing_event_##IDX;                                                   \
		t->source.add_sink = timing_add_sink_##IDX;                                                            \
		t->source.remove_sink = timing_remove_sink_##IDX;                                                      \
	} while (false);


/*
 *
 * Frame node functions.
 *
 */

static void
send_break_apart(struct xrt_frame_node *node)
{
	struct b_sink_bridge *bsb = container_of(node, struct b_sink_bridge, send_node);

	os_rwlock_write_lock(&bsb->lock);
	bsb->sending = false;
	os_rwlock_unlock(&bsb->lock);
}

static void
send_destroy(struct xrt_frame_node *node)
{
	bridge_unref(container_of(node, struct b_sink_bridge, send_node));
}

static void
recv_break_apart(struct xrt_frame_node *node)
{
	struct b_sink_bridge *bsb = container_of(node, struct b_sink_bridge, recv_node);

	os_rwlock_write_lock(&bsb->lock);
	bsb->receiving = false;
	for (size_t i = 0; i < ARRAY_SIZE(bsb->frames); i++) {
		U_ZERO_ARRAY(bsb->frames[i].sinks);
	}
	for (size_t i = 0; i < ARRAY_SIZE(bsb->timings); i++) {
		U_ZERO_ARRAY(bsb->timings[i].sinks);
	}
	os_rwlock_unlock(&bsb->lock);
}

static void
recv_destroy(struct xrt_frame_node *node)
{
	struct b_sink_bridge *bsb = container_of(node, struct b_sink_bridge, recv_node);

	os_rwlock_write_lock(&bsb->lock);
	bsb->recv_in_context = false;
	os_rwlock_unlock(&bsb->lock);

	bridge_unref(bsb);
}


/*
 *
 * 'Exported' functions.
 *
 */

xrt_result_t
b_sink_bridge_create(struct xrt_frame_context *send_xfctx, struct b_sink_bridge **out_bsb)
{
	struct b_sink_bridge *bsb = U_TYPED_CALLOC(struct b_sink_bridge);
	if (bsb == NULL) {
		return XRT_ERROR_ALLOCATION;
	}

	if (os_rwlock_init(&bsb->lock) != 0) {
		free(bsb);
		return XRT_ERROR_SYNC_PRIMITIVE_CREATION_FAILED;
	}

	XRT_TRACKING_FOR_EACH_FRAME_SOURCE(SETUP_FRAME)
	XRT_TRACKING_FOR_EACH_FRAME_SOURCE(SETUP_TIMING)
#undef SETUP_FRAME
#undef SETUP_TIMING

	bsb->ref.count = 1;
	bsb->sending = true;
	bsb->send_node.break_apart = send_break_apart;
	bsb->send_node.destroy = send_destroy;
	bsb->recv_node.break_apart = recv_break_apart;
	bsb->recv_node.destroy = recv_destroy;

	xrt_frame_context_add(send_xfctx, &bsb->send_node);

	*out_bsb = bsb;

	return XRT_SUCCESS;
}

xrt_result_t
b_sink_bridge_attach(struct b_sink_bridge *bsb, struct xrt_frame_context *recv_xfctx)
{
	os_rwlock_write_lock(&bsb->lock);
	if (bsb->recv_in_context) {
		os_rwlock_unlock(&bsb->lock);
		return XRT_ERROR_INVALID_ARGUMENT;
	}

	xrt_reference_inc(&bsb->ref);
	bsb->receiving = true;
	bsb->recv_in_context = true;
	xrt_frame_context_add(recv_xfctx, &bsb->recv_node);
	os_rwlock_unlock(&bsb->lock);

	return XRT_SUCCESS;
}

struct xrt_frame_sink *
b_sink_bridge_get_frame_sink(struct b_sink_bridge *bsb, uint32_t index)
{
	assert(index < ARRAY_SIZE(bsb->frames));
	return &bsb->frames[index].sink;
}

struct t_timing_event_sink *
b_sink_bridge_get_timing_sink(struct b_sink_bridge *bsb, uint32_t index)
{
	assert(index < ARRAY_SIZE(bsb->timings));
	return &bsb->timings[index].sink;
}

struct xrt_frame_source *
b_sink_bridge_get_frame_source(struct b_sink_bridge *bsb, uint32_t index)
{
	assert(index < ARRAY_SIZE(bsb->frames));
	return &bsb->frames[index].source;
}

struct t_timing_event_source *
b_sink_bridge_get_timing_source(struct b_sink_bridge *bsb, uint32_t index)
{
	assert(index < ARRAY_SIZE(bsb->timings));
	return &bsb->timings[index].source;
}
