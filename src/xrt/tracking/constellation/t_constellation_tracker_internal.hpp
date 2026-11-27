// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal ures for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_var.h"
#include "util/u_threading.h"
#include "util/u_weak_ptr.hpp"
#include "util/u_sink.h"
#include "util/u_frame.h"
#include "util/u_frame_scribble.h"

#include "glue/g_blob_sink.hpp"
#include "glue/g_frame_node.hpp"
#include "glue/g_imu_sink.hpp"

#include "tracking/t_constellation.h"

#include "math/m_api.h"

#include "correspondence_search.h"
#include "led_search_model.h"
#include "pose_metrics.h"
#include "t_constellation_tracker.h"
#include "optimizer/pose_optimize.hpp"
#include "optimizer/rig_calibration.hpp"
#include "optimizer/sensor_fusion.hpp"

#include <array>
#include <atomic>
#include <ceres/context.h>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <span>
#include <stdexcept>
#include <vector>


#define CT_TRACE(ct, ...) U_LOG_IFL_T(ct->log_level, __VA_ARGS__)
#define CT_DEBUG(ct, ...) U_LOG_IFL_D(ct->log_level, __VA_ARGS__)
#define CT_INFO(ct, ...) U_LOG_IFL_I(ct->log_level, __VA_ARGS__)
#define CT_WARN(ct, ...) U_LOG_IFL_W(ct->log_level, __VA_ARGS__)
#define CT_ERROR(ct, ...) U_LOG_IFL_E(ct->log_level, __VA_ARGS__)

#define MIN_ROT_ERROR DEG_TO_RAD(30)
#define MIN_POS_ERROR 0.10

/*
 *
 * Forward declares for C callback functions
 *
 */

extern "C" void *
constellation_tracker_camera_slow_thread(void *ptr);

extern "C" void *
constellation_tracker_camera_fast_thread(void *ptr);


namespace xrt::tracking::constellation {

using namespace xrt::tracking::constellation::optimizer;

namespace os = xrt::auxiliary::os;

// Forward-declares
struct Camera;
struct CameraMosaic;
struct ConstellationTracker;
struct DataRecorder;
struct Device;

struct FoundDevicePose
{
	xrt_pose Tcv_cam_device XRT_POSE_IDENTITY;
	float average_blob_brightness;

	/*!
	 * Whether a found pose was already handled, this is present so that if a single sample is torn between some
	 * devices fast recovering and some devices slow recovering, we push the fast recovery poses the moment they're
	 * found rather than at the end of the slow recovery where the sample is "terminated".
	 */
	bool handled{false};
};

struct DeviceState
{
	//! The ID of the device.
	t_constellation_device_id_t device_id{XRT_CONSTELLATION_INVALID_DEVICE_ID};

	//! The predicted pose of the device in the world at the time of the sample.
	std::optional<xrt_pose> Txr_world_device_prior{std::nullopt};
	//! The predicted pose of the device relative to the camera at the time of the sample.
	std::optional<xrt_space_relation> Tcv_cam_device_predicted{std::nullopt};

	//! The final found pose of the device in this specific sample.
	std::optional<FoundDevicePose> found_pose{std::nullopt};

	//! Whether the device needs to have the slow processing thread run over it
	bool needs_slow_processing{false};
};

struct CameraSample
{
public: // Fields
	t_blobwatch *source{nullptr};
	uint64_t id{};
	uint64_t sequence_id{};
	int64_t timestamp_ns{};
	t_blob blobs[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME]{};
	uint32_t blob_count{};

	//! The camera's position in the constellation tracker's tracking origin
	std::optional<xrt_pose> Txr_world_cam{std::nullopt};

	std::array<t_constellation_device_id_t, XRT_CONSTELLATION_MAX_DEVICES> device_ids_by_distance{};
	std::array<DeviceState, XRT_CONSTELLATION_MAX_DEVICES> device_states{};
	uint32_t device_count{};

	uint32_t mosaic_index;
	uint32_t camera_index;

public: // Methods
	std::optional<DeviceState *>
	getDeviceState(t_constellation_device_id_t device_id);

	DeviceState &
	putDeviceState(t_constellation_device_id_t device_id);

	CameraSample(t_blob_observation &blobservation, Camera *camera);

	CameraSample() = default;

	void
	markMatchingBlobs(ConstellationTracker *ct,
	                  t_constellation_tracker_led_model &led_model,
	                  t_constellation_device_id_t device_id,
	                  pose_metrics_blob_match_info &blob_match_info);

	t_blob_observation
	toBlobObservation() const &
	{
		t_blob_observation obs = {
		    .source = this->source,
		    .id = this->id,
		    .sequence_id = this->sequence_id,
		    .timestamp_ns = this->timestamp_ns,
		    .blobs = const_cast<t_blob *>(this->blobs),
		    .num_blobs = this->blob_count,
		    .frame = nullptr,
		};

		return obs;
	}

	t_blob_observation
	toBlobObservation() && = delete;
	t_blob_observation
	toBlobObservation() const && = delete;
};

struct CameraScribbleSettings
{
public: // Fields
	//! Whether to draw the blobs in the sample
	bool draw_blobs{true};
	//! Whether to draw the raw blob IDs
	bool draw_blob_ids{false};
	//! Whether to draw the device IDs on the blobs where possible
	bool draw_blob_device_ids{true};
	//! Whether to draw the LED IDs on the blobs where possible
	bool draw_blob_led_ids{false};

	//! Whether to draw the prior device pose in the sample, if it exists.
	bool draw_prior{false};
	//! Whether to draw the final device pose in the sample, if it exists.
	bool draw_found{false};

public: // Methods
	void
	setupDebugTracking(void *root);
};

struct Camera : util::BlobSinkBase<Camera>
{
public: // Fields
	//! The owner tracker, so we can retrieve it from the blob sink callback
	ConstellationTracker *tracker;
	std::weak_ptr<CameraMosaic> mosaic;

	t_camera_calibration calibration;

	camera_model model;

	CameraScribbleSettings scribble_settings{};

	//! The index in the mosaic
	uint32_t index;

	//! Does "slow" processing for this camera when fast recovery paths fail.
	os_thread_helper slow_processing_thread{};
	struct
	{
		std::optional<CameraSample> sample{std::nullopt};

		correspondence_search *cs{nullptr};

		u_sink_debug debug_sink{};
	} slow_processing_thread_data;

	/*!
	 * Does "fast" processing for this camera, trying to recover a pose quickly. It's valid for this to happen at
	 * the same time as a slow process.
	 */
	os_thread_helper fast_processing_thread{};
	struct
	{
		std::optional<CameraSample> sample{std::nullopt};

		correspondence_search *cs{nullptr};

		u_sink_debug debug_sink{};
	} fast_processing_thread_data;

	//! Locks all processing data
	mutable os::Mutex processing_lock;
	//! All data protected by the processing lock
	struct
	{
		xrt_pose Txr_origin_cam;
		bool has_concrete_pose;
	} locked_data;

	std::atomic<uint32_t> shallow_searches_failed{0};

public: // Methods (t_blob_sink, t_constellation_tracker.cpp)
	void
	pushBlobs(t_blob_observation &observation);

	static void
	destroyBlobSink(t_blob_sink *sink);

public: // Methods (t_constellation_tracker.cpp)
	Camera(ConstellationTracker *tracker,
	       std::weak_ptr<CameraMosaic> mosaic,
	       const t_constellation_tracker_camera &camera_params,
	       enum u_logging_level *log_level_ptr,
	       uint32_t index);

	~Camera();

	// Delete all copy/move ctors, since the pointers we pass out need to be stable
	Camera(const Camera &) = delete;
	Camera(Camera &&) = delete;
	Camera &
	operator=(const Camera &) = delete;
	Camera &
	operator=(Camera &&) = delete;

	std::optional<xrt_pose>
	getWorldPose(timepoint_ns when_ns);

	void
	deferSampleToSlowThread(CameraSample &sample);

	//! Fast matching based on prior pose
	bool
	tryDevicePose(Device *device,
	              CameraSample &sample,
	              DeviceState &device_state,
	              bool need_matching_ids,
	              const std::optional<xrt_space_relation> &Tcv_cam_device_prior,
	              const xrt_pose &Tcv_cam_device_candidate);

	bool
	tryDeviceBlobRecovery(Device *device,
	                      CameraSample &sample,
	                      DeviceState &device_state,
	                      const std::optional<xrt_space_relation> &Tcv_cam_device_prior);

	//! Handles a single step (fast,slow) being complete
	void
	handleStepComplete(CameraSample &sample);

	//! Handles complete processing of a sample, hold the devices lock while you call this.
	void
	handleProcessingComplete(CameraSample &sample, const std::span<Device *> devices);

	void
	processSampleSlow(CameraSample &sample);

	//! Returns whether a slow search is needed
	bool
	processSampleFast(CameraSample &sample);

	void
	pushPose(CameraSample &camera_sample,
	         DeviceState &device_state,
	         Device *device,
	         pose_metrics &score,
	         const xrt_pose &Tcv_cam_device,
	         bool was_optimized);

public: // Methods (constellation_debug_scribble.cpp)
	void
	debugScribbleSample(CameraSample &sample, bool fast);
};

struct CameraMosaic
{
public: // Fields
	std::vector<std::unique_ptr<Camera>> cameras;
	//! The index of this mosaic in the tracker.
	uint32_t index;

	t_constellation_tracker_tracking_source *tracking_origin;

public: // Methods
	CameraMosaic(ConstellationTracker *tracker,
	             const t_constellation_tracker_camera_mosaic &mosaic_params,
	             uint32_t index);

	~CameraMosaic() = default;

	std::optional<xrt_pose>
	getTrackingOriginPose(timepoint_ns when_ns);
};

struct DeviceLastPose
{
public: // Fields
	xrt_pose Txr_world_device;
	xrt_pose Tcv_cam_device;
	timepoint_ns timestamp_ns;

public: // Methods
	DeviceLastPose(xrt_pose Txr_world_device, xrt_pose Tcv_cam_device, timepoint_ns timestamp_ns);
};

struct Device : public util::ImuSinkBase<Device>
{
public: // Fields
	t_constellation_tracker_device_params params;
	t_constellation_tracker_device *device;

	t_constellation_device_id_t id;

	//! Devices are disabled by default, and the constellation tracker must choose to enable them.
	std::atomic<bool> enabled{false};

	//! The owner tracker, so we can retrieve it from the IMU sink callback
	ConstellationTracker *tracker;

	std::unique_ptr<sensor_fusion::SensorFusion> sensor_fusion{};

	// @todo remove when clang-format is updated in CI
	// clang-format off
	t_constellation_search_model *search_model{nullptr};

	// @todo These need to be pulled from the device and put into the sample. 
	//       Right now we just hardcode them since we don't have any real sensor fusion.
	xrt_vec3 prior_pos_error{MIN_POS_ERROR, MIN_POS_ERROR, MIN_POS_ERROR};
	xrt_vec3 prior_rot_error{MIN_ROT_ERROR, MIN_ROT_ERROR, MIN_ROT_ERROR};
	float gravity_error_rad{MIN_ROT_ERROR}; /* Gravity vector uncertainty in radians 0..M_PI */

	mutable os::Mutex data_lock;
	struct
	{
		std::optional<DeviceLastPose> last_known_pose;

		struct
		{
			//! The latest sequence ID this device was seen at.
			std::optional<int64_t> last_seen_sequence_id{std::nullopt};
		} per_camera[XRT_CONSTELLATION_MAX_TRACKING_MOSAICS * XRT_TRACKING_MAX_FRAME_SOURCES];
	} locked_data;
	// clang-format on

public: // Methods
	Device(ConstellationTracker *tracker,
	       t_constellation_tracker_device_params *params,
	       t_constellation_tracker_device *device,
	       t_constellation_device_id_t id);

	~Device();

	// Delete move constructors
	Device(const Device &) = delete;
	Device(Device &&) = delete;
	Device &
	operator=(const Device &) = delete;
	Device &
	operator=(Device &&) = delete;

	//! @note The interface expects non-const but we don't need to mutate it, it's source-compatible
	void
	pushImu(const xrt_imu_sample &sample);
};

struct ConstellationTracker : public util::FrameNodeBase<ConstellationTracker>
{
public: // Fields
	xrt_tracking_origin tracking_origin = {};

	//! Whether the constellation tracker is running
	bool running = true;

	enum u_logging_level log_level;

	t_constellation_tracker_params params;

	bool deterministic{false};

	std::vector<std::shared_ptr<CameraMosaic>> mosaics;

	// Put here to be destroyed before devices (reverse declaration order)
	std::unique_ptr<ceres::Context> ceres_context{};

	std::shared_mutex device_lock;
	std::vector<std::unique_ptr<Device>> devices;
	t_constellation_device_id_t next_device_id{0};

	std::unique_ptr<DataRecorder> data_recorder{};

	mutable os::Mutex rig_calibration_lock{};
	os_thread_helper rig_calibration_thread{};
	bool running_calibration{false};
	std::unique_ptr<rig_calibration::RigCalibration> rig_calibration{};

#ifdef XRT_FEATURE_RERUN
	std::unique_ptr<struct RerunContext> rerun_stream{};
#endif

public: // Methods (xrt_frame_node)
	void
	breakApart();

	static void
	destroyFrameNode(xrt_frame_node *node);

public: // Methods
	static ConstellationTracker *
	Get(t_constellation_tracker *tracker)
	{
		return reinterpret_cast<ConstellationTracker *>(tracker);
	}

	ConstellationTracker(t_constellation_tracker_params *params);

	~ConstellationTracker();

	// Delete all copy/move ctors, since the pointers we return need to be stable
	ConstellationTracker(const ConstellationTracker &) = delete;
	ConstellationTracker(ConstellationTracker &&) = delete;
	ConstellationTracker &
	operator=(const ConstellationTracker &) = delete;
	ConstellationTracker &
	operator=(ConstellationTracker &&) = delete;

	bool
	needSensorCalibration();

	bool
	checkSensorCalibration();

	void
	setupVariableTracking();

	t_constellation_device_id_t
	addDevice(t_constellation_tracker_device_params *params, t_constellation_tracker_device *device);

	void
	removeDevice(t_constellation_device_id_t device_id);

	void
	getTrackedPose(t_constellation_device_id_t device_id, timepoint_ns when_ns, xrt_space_relation &out_relation);
};

}; // namespace xrt::tracking::constellation
