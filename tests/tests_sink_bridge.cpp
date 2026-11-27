// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  b_sink_bridge and b_frame_source tests.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 */

#include "b_frame_source.h"
#include "b_sink_bridge.h"

#include "catch_amalgamated.hpp"


namespace {

struct CountingFrameSink
{
	xrt_frame_sink base = {};
	int count = 0;

	CountingFrameSink()
	{
		base.push_frame = [](xrt_frame_sink *xs, xrt_frame *) {
			reinterpret_cast<CountingFrameSink *>(xs)->count++;
		};
	}
};

struct CountingTimingSink
{
	t_timing_event_sink base = {};
	int count = 0;

	CountingTimingSink()
	{
		base.push_timing_event = [](t_timing_event_sink *sink, const t_timing_event *) {
			reinterpret_cast<CountingTimingSink *>(sink)->count++;
		};
	}
};

void
push(xrt_frame_sink *xs)
{
	xrt_frame xf = {};
	xrt_sink_push_frame(xs, &xf);
}

//! Pushes a frame when broken apart.
struct ProducerNode
{
	xrt_frame_node node = {};
	xrt_frame_sink *sink = nullptr;
	bool pushed = false;

	ProducerNode()
	{
		node.break_apart = [](xrt_frame_node *node) {
			ProducerNode *p = reinterpret_cast<ProducerNode *>(node);
			push(p->sink);
			p->pushed = true;
		};
		node.destroy = [](xrt_frame_node *) {};
	}
};

void
push(t_timing_event_sink *sink)
{
	t_timing_event event = {};
	t_timing_event_sink_push_timing_event(sink, &event);
}

} // namespace


TEST_CASE("b_frame_source")
{
	xrt_frame_context xfctx = {};
	xrt_frame_sink *in = nullptr;
	xrt_frame_source *source = nullptr;
	REQUIRE(b_frame_source_create(&xfctx, &in, &source) == XRT_SUCCESS);

	CountingFrameSink a, b;
	CHECK(xrt_frame_source_add_sink(source, &a.base) == XRT_SUCCESS);
	CHECK(xrt_frame_source_add_sink(source, &b.base) == XRT_SUCCESS);
	push(in);
	CHECK(a.count == 1);
	CHECK(b.count == 1);

	xrt_frame_source_remove_sink(source, &a.base);
	push(in);
	CHECK(a.count == 1);
	CHECK(b.count == 2);

	xrt_frame_context_destroy_nodes(&xfctx);
}

TEST_CASE("b_sink_bridge")
{
	xrt_frame_context send = {};
	xrt_frame_context recv = {};

	// Added before the bridge so it's broken apart after it, like a producer that is still pushing.
	ProducerNode producer;
	xrt_frame_context_add(&send, &producer.node);

	b_sink_bridge *bsb = nullptr;
	REQUIRE(b_sink_bridge_create(&send, &bsb) == XRT_SUCCESS);

	// Frame and timing sources are indexed separately, use different ones.
	xrt_frame_sink *frame_in = b_sink_bridge_get_frame_sink(bsb, 2);
	producer.sink = frame_in;
	t_timing_event_sink *timing_in = b_sink_bridge_get_timing_sink(bsb, 1);
	CountingFrameSink frames;
	CountingTimingSink timings;

	// Nothing to add to before attach.
	CHECK(xrt_frame_source_add_sink(b_sink_bridge_get_frame_source(bsb, 2), &frames.base) != XRT_SUCCESS);
	CHECK(t_timing_event_source_add_sink(b_sink_bridge_get_timing_source(bsb, 1), &timings.base) != 0);
	push(frame_in);

	REQUIRE(b_sink_bridge_attach(bsb, &recv) == XRT_SUCCESS);
	CHECK(b_sink_bridge_attach(bsb, &recv) != XRT_SUCCESS);
	CHECK(xrt_frame_source_add_sink(b_sink_bridge_get_frame_source(bsb, 2), &frames.base) == XRT_SUCCESS);
	CHECK(t_timing_event_source_add_sink(b_sink_bridge_get_timing_source(bsb, 1), &timings.base) == 0);

	push(frame_in);
	push(timing_in);
	push(b_sink_bridge_get_frame_sink(bsb, 1));  // Other frame source.
	push(b_sink_bridge_get_timing_sink(bsb, 2)); // Other timing source.
	CHECK(frames.count == 1);
	CHECK(timings.count == 1);

	SECTION("receiving context restarted")
	{
		xrt_frame_context_destroy_nodes(&recv);
		push(frame_in);
		push(timing_in);
		CHECK(frames.count == 1);
		CHECK(timings.count == 1);

		// Sinks were dropped with the old context.
		REQUIRE(b_sink_bridge_attach(bsb, &recv) == XRT_SUCCESS);
		push(frame_in);
		CHECK(frames.count == 1);

		CHECK(xrt_frame_source_add_sink(b_sink_bridge_get_frame_source(bsb, 2), &frames.base) == XRT_SUCCESS);
		push(frame_in);
		CHECK(frames.count == 2);

		xrt_frame_context_destroy_nodes(&recv);
		xrt_frame_context_destroy_nodes(&send);
	}

	SECTION("sending context destroyed first")
	{
		xrt_frame_context_destroy_nodes(&send);
		CHECK(producer.pushed);
		CHECK(frames.count == 1);
		xrt_frame_context_destroy_nodes(&recv);
	}
}

TEST_CASE("b_sink_bridge every index")
{
	xrt_frame_context send = {};
	xrt_frame_context recv = {};
	b_sink_bridge *bsb = nullptr;
	REQUIRE(b_sink_bridge_create(&send, &bsb) == XRT_SUCCESS);
	REQUIRE(b_sink_bridge_attach(bsb, &recv) == XRT_SUCCESS);

	CountingFrameSink frames[B_SINK_BRIDGE_MAX_FRAME_SOURCES];
	for (uint32_t i = 0; i < B_SINK_BRIDGE_MAX_FRAME_SOURCES; i++) {
		CHECK(xrt_frame_source_add_sink(b_sink_bridge_get_frame_source(bsb, i), &frames[i].base) ==
		      XRT_SUCCESS);
		for (uint32_t k = 0; k <= i; k++) {
			push(b_sink_bridge_get_frame_sink(bsb, i));
		}
	}

	CountingTimingSink timings[B_SINK_BRIDGE_MAX_TIMING_SOURCES];
	for (uint32_t i = 0; i < B_SINK_BRIDGE_MAX_TIMING_SOURCES; i++) {
		CHECK(t_timing_event_source_add_sink(b_sink_bridge_get_timing_source(bsb, i), &timings[i].base) == 0);
		for (uint32_t k = 0; k <= i; k++) {
			push(b_sink_bridge_get_timing_sink(bsb, i));
		}
	}

	for (uint32_t i = 0; i < B_SINK_BRIDGE_MAX_FRAME_SOURCES; i++) {
		CHECK(frames[i].count == int(i + 1));
	}
	for (uint32_t i = 0; i < B_SINK_BRIDGE_MAX_TIMING_SOURCES; i++) {
		CHECK(timings[i].count == int(i + 1));
	}

	xrt_frame_context_destroy_nodes(&recv);
	xrt_frame_context_destroy_nodes(&send);
}

TEST_CASE("b_sink_bridge never attached")
{
	xrt_frame_context send = {};
	b_sink_bridge *bsb = nullptr;
	REQUIRE(b_sink_bridge_create(&send, &bsb) == XRT_SUCCESS);
	xrt_frame_context_destroy_nodes(&send);
}
