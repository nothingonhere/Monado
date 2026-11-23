// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Base frame source.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup base
 */

#include "util/u_misc.h"

#include "os/os_threading.h"

#include "b_frame_source.h"


#define B_FRAME_SOURCE_MAX_SINKS 4

struct b_frame_source
{
	struct xrt_frame_source source;
	struct xrt_frame_sink sink;
	struct xrt_frame_node node;

	struct os_mutex mutex;

	bool running;

	struct xrt_frame_sink *sinks[B_FRAME_SOURCE_MAX_SINKS];
};


/*
 *
 * xrt_frame_source functions.
 *
 */

static struct b_frame_source *
from_source(struct xrt_frame_source *xfsrc)
{
	return container_of(xfsrc, struct b_frame_source, source);
}

static xrt_result_t
b_frame_source_add_sink(struct xrt_frame_source *xfsrc, struct xrt_frame_sink *xs)
{
	struct b_frame_source *bfs = from_source(xfsrc);
	xrt_result_t xret = XRT_ERROR_ALLOCATION;

	os_mutex_lock(&bfs->mutex);
	for (size_t i = 0; i < ARRAY_SIZE(bfs->sinks); i++) {
		if (bfs->sinks[i] == NULL) {
			bfs->sinks[i] = xs;
			xret = XRT_SUCCESS;
			break;
		}
	}
	os_mutex_unlock(&bfs->mutex);

	return xret;
}

static void
b_frame_source_remove_sink(struct xrt_frame_source *xfsrc, struct xrt_frame_sink *xs)
{
	struct b_frame_source *bfs = from_source(xfsrc);

	os_mutex_lock(&bfs->mutex);
	for (size_t i = 0; i < ARRAY_SIZE(bfs->sinks); i++) {
		if (bfs->sinks[i] == xs) {
			bfs->sinks[i] = NULL;
		}
	}
	os_mutex_unlock(&bfs->mutex);
}


/*
 *
 * xrt_frame_sink functions.
 *
 */

static struct b_frame_source *
from_sink(struct xrt_frame_sink *xs)
{
	return container_of(xs, struct b_frame_source, sink);
}

/*!
 * The lock is held while pushing, sinks are not reference counted so this is
 * what stops remove_sink from returning while a push to the sink is in flight.
 */
static void
b_frame_source_push_frame(struct xrt_frame_sink *xs, struct xrt_frame *xf)
{
	struct b_frame_source *bfs = from_sink(xs);

	os_mutex_lock(&bfs->mutex);
	if (bfs->running) {
		for (size_t i = 0; i < ARRAY_SIZE(bfs->sinks); i++) {
			if (bfs->sinks[i] != NULL) {
				xrt_sink_push_frame(bfs->sinks[i], xf);
			}
		}
	}
	os_mutex_unlock(&bfs->mutex);
}


/*
 *
 * xrt_frame_node functions.
 *
 */

static struct b_frame_source *
from_node(struct xrt_frame_node *node)
{
	return container_of(node, struct b_frame_source, node);
}

static void
b_frame_source_break_apart(struct xrt_frame_node *node)
{
	struct b_frame_source *bfs = from_node(node);

	os_mutex_lock(&bfs->mutex);
	bfs->running = false;
	os_mutex_unlock(&bfs->mutex);
}

static void
b_frame_source_destroy(struct xrt_frame_node *node)
{
	struct b_frame_source *bfs = from_node(node);

	os_mutex_destroy(&bfs->mutex);

	free(bfs);
}


/*
 *
 * 'Exported' functions.
 *
 */

xrt_result_t
b_frame_source_create(struct xrt_frame_context *xfctx,
                      struct xrt_frame_sink **out_sink,
                      struct xrt_frame_source **out_source)
{
	struct b_frame_source *bfs = U_TYPED_CALLOC(struct b_frame_source);
	if (bfs == NULL) {
		return XRT_ERROR_ALLOCATION;
	}

	if (os_mutex_init(&bfs->mutex) < 0) {
		free(bfs);
		return XRT_ERROR_SYNC_PRIMITIVE_CREATION_FAILED;
	}

	bfs->source.add_sink = b_frame_source_add_sink;
	bfs->source.remove_sink = b_frame_source_remove_sink;
	bfs->sink.push_frame = b_frame_source_push_frame;
	bfs->node.break_apart = b_frame_source_break_apart;
	bfs->node.destroy = b_frame_source_destroy;
	bfs->running = true;

	xrt_frame_context_add(xfctx, &bfs->node);

	*out_sink = &bfs->sink;
	*out_source = &bfs->source;

	return XRT_SUCCESS;
}
