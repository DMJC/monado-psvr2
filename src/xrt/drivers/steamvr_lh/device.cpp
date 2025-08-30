// Copyright 2023, Shawn Wallace
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief SteamVR driver device implementation.
 * @author Shawn Wallace <yungwallace@live.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_steamvr_lh
 */

#include "math/m_api.h"
#include "math/m_relation_history.h"
#include "math/m_space.h"

#include "interfaces/context.hpp"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_hand_simulation.h"
#include "util/u_hand_tracking.h"
#include "util/u_logging.h"
#include "util/u_json.hpp"

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"

#include "vive/vive_poses.h"

#include "openvr_driver.h"

#include "device.hpp"

#include <cmath>
#include <functional>
#include <cstring>
#include <thread>
#include <algorithm>
#include <map>


#define DEV_ERR(...) U_LOG_IFL_E(ctx->log_level, __VA_ARGS__)
#define DEV_WARN(...) U_LOG_IFL_W(ctx->log_level, __VA_ARGS__)
#define DEV_INFO(...) U_LOG_IFL_I(ctx->log_level, __VA_ARGS__)
#define DEV_DEBUG(...) U_LOG_IFL_D(ctx->log_level, __VA_ARGS__)

DEBUG_GET_ONCE_BOOL_OPTION(lh_emulate_hand, "LH_EMULATE_HAND", true)

// Each device will have its own input class.
struct InputClass
{
	xrt_device_name name;
	const std::vector<xrt_input_name> poses;
	const std::unordered_map<std::string_view, xrt_input_name> non_poses;
	const std::unordered_map<std::string_view, IndexFinger> finger_curls;
};

namespace {
// Adding support for a new controller is a simple as adding it here.
// The key for the map needs to be the name of input profile as indicated by the lighthouse driver.
const std::unordered_map<std::string_view, InputClass> controller_classes{
    {
        "vive_controller",
        InputClass{
            XRT_DEVICE_VIVE_WAND,
            {
                XRT_INPUT_VIVE_GRIP_POSE,
                XRT_INPUT_VIVE_AIM_POSE,
            },
            {
                {"/input/application_menu/click", XRT_INPUT_VIVE_MENU_CLICK},
                {"/input/trackpad/click", XRT_INPUT_VIVE_TRACKPAD_CLICK},
                {"/input/trackpad/touch", XRT_INPUT_VIVE_TRACKPAD_TOUCH},
                {"/input/system/click", XRT_INPUT_VIVE_SYSTEM_CLICK},
                {"/input/trigger/click", XRT_INPUT_VIVE_TRIGGER_CLICK},
                {"/input/trigger/value", XRT_INPUT_VIVE_TRIGGER_VALUE},
                {"/input/grip/click", XRT_INPUT_VIVE_SQUEEZE_CLICK},
                {"/input/trackpad", XRT_INPUT_VIVE_TRACKPAD},
            },
            {
                // No fingers on this controller type
            },
        },
    },
    {
        "index_controller",
        InputClass{
            XRT_DEVICE_INDEX_CONTROLLER,
            {
                XRT_INPUT_INDEX_GRIP_POSE,
                XRT_INPUT_INDEX_AIM_POSE,
            },
            {
                {"/input/system/click", XRT_INPUT_INDEX_SYSTEM_CLICK},
                {"/input/system/touch", XRT_INPUT_INDEX_SYSTEM_TOUCH},
                {"/input/a/click", XRT_INPUT_INDEX_A_CLICK},
                {"/input/a/touch", XRT_INPUT_INDEX_A_TOUCH},
                {"/input/b/click", XRT_INPUT_INDEX_B_CLICK},
                {"/input/b/touch", XRT_INPUT_INDEX_B_TOUCH},
                {"/input/trigger/click", XRT_INPUT_INDEX_TRIGGER_CLICK},
                {"/input/trigger/touch", XRT_INPUT_INDEX_TRIGGER_TOUCH},
                {"/input/trigger/value", XRT_INPUT_INDEX_TRIGGER_VALUE},
                {"/input/grip/force", XRT_INPUT_INDEX_SQUEEZE_FORCE},
                {"/input/grip/value", XRT_INPUT_INDEX_SQUEEZE_VALUE},
                {"/input/thumbstick/click", XRT_INPUT_INDEX_THUMBSTICK_CLICK},
                {"/input/thumbstick/touch", XRT_INPUT_INDEX_THUMBSTICK_TOUCH},
                {"/input/thumbstick", XRT_INPUT_INDEX_THUMBSTICK},
                {"/input/trackpad/force", XRT_INPUT_INDEX_TRACKPAD_FORCE},
                {"/input/trackpad/touch", XRT_INPUT_INDEX_TRACKPAD_TOUCH},
                {"/input/trackpad", XRT_INPUT_INDEX_TRACKPAD},
            },
            {
                {"/input/finger/index", IndexFinger::Index},
                {"/input/finger/middle", IndexFinger::Middle},
                {"/input/finger/ring", IndexFinger::Ring},
                {"/input/finger/pinky", IndexFinger::Pinky},
            },
        },
    },
    {
        "vive_tracker",
        InputClass{
            XRT_DEVICE_VIVE_TRACKER,
            {
                XRT_INPUT_GENERIC_TRACKER_POSE,
            },
            {
                {"/input/power/click", XRT_INPUT_VIVE_TRACKER_SYSTEM_CLICK},
                {"/input/grip/click", XRT_INPUT_VIVE_TRACKER_SQUEEZE_CLICK},
                {"/input/application_menu/click", XRT_INPUT_VIVE_TRACKER_MENU_CLICK},
                {"/input/trigger/click", XRT_INPUT_VIVE_TRACKER_TRIGGER_CLICK},
                {"/input/thumb/click", XRT_INPUT_VIVE_TRACKER_TRACKPAD_CLICK},
            },
            {
                // No fingers on this controller type
            },
        },
    },
    {
        "tundra_tracker",
        InputClass{
            XRT_DEVICE_VIVE_TRACKER,
            {
                XRT_INPUT_GENERIC_TRACKER_POSE,
            },
            {
                {"/input/power/click", XRT_INPUT_VIVE_TRACKER_SYSTEM_CLICK},
                {"/input/grip/click", XRT_INPUT_VIVE_TRACKER_SQUEEZE_CLICK},
                {"/input/application_menu/click", XRT_INPUT_VIVE_TRACKER_MENU_CLICK},
                {"/input/trigger/click", XRT_INPUT_VIVE_TRACKER_TRIGGER_CLICK},
                {"/input/thumb/click", XRT_INPUT_VIVE_TRACKER_TRACKPAD_CLICK},
            },
            {
                // No fingers on this controller type
            },
        },
    },
};

int64_t
chrono_timestamp_ns()
{
	auto now = std::chrono::steady_clock::now().time_since_epoch();
	int64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
	return ts;
}

// Template for calling a member function of Device from a free function
template <typename DeviceType, auto Func, typename Ret, typename... Args>
inline Ret
device_bouncer(struct xrt_device *xdev, Args... args)
{
	auto *dev = static_cast<DeviceType *>(xdev);
	return std::invoke(Func, dev, args...);
}

// Setting used for brightness in the steamvr section. This isn't defined by the openvr header.
static const char *analog_gain_settings_key = "analogGain";

/**
 * Map a 0-1 (or > 1) brightness value into the analogGain value stored in SteamVR settings.
 */
float
brightness_to_analog_gain(float brightness)
{
	// Lookup table from brightness to analog gain value
	// Courtesy of OyasumiVR, MIT license, Copyright (c) 2022 Raphiiko
	// https://github.com/Raphiiko/OyasumiVR/blob/c9e7fbcc2ea6caa07a8233a75218598087043171/src-ui/app/services/brightness-control/hardware-brightness-drivers/valve-index-hardware-brightness-control-driver.ts#L92
	// TODO: We should support having a lookup table per headset model. If not present, fallback to lerp between the
	// given min and max analog gain. Maybe we can assume 100% brightness = 1.0 analog gain, but we need info from
	// more headsets.
	static const auto lookup = std::map<float, float>{{
	    {0.20, 0.03},  {0.23, 0.04},  {0.26, 0.05},  {0.27, 0.055}, {0.28, 0.06},  {0.30, 0.07},  {0.32, 0.08},
	    {0.33, 0.09},  {0.34, 0.095}, {0.35, 0.1},   {0.36, 0.105}, {0.37, 0.11},  {0.37, 0.115}, {0.38, 0.12},
	    {0.39, 0.125}, {0.40, 0.13},  {0.40, 0.135}, {0.41, 0.14},  {0.42, 0.145}, {0.42, 0.15},  {0.43, 0.155},
	    {0.43, 0.16},  {0.44, 0.165}, {0.45, 0.17},  {0.45, 0.175}, {0.46, 0.18},  {0.46, 0.185}, {0.47, 0.19},
	    {0.48, 0.195}, {0.48, 0.2},   {0.49, 0.21},  {0.53, 0.25},  {0.58, 0.3},   {0.59, 0.315}, {0.60, 0.32},
	    {0.60, 0.33},  {0.61, 0.34},  {0.62, 0.35},  {0.66, 0.4},   {0.69, 0.445}, {0.70, 0.45},  {0.70, 0.46},
	    {0.71, 0.465}, {0.71, 0.47},  {0.71, 0.475}, {0.72, 0.48},  {0.72, 0.49},  {0.73, 0.5},   {0.79, 0.6},
	    {0.85, 0.7},   {0.90, 0.8},   {0.95, 0.9},   {1.00, 1},     {1.50, 1.50},
	}};

	if (const auto upper_it = lookup.upper_bound(brightness); upper_it == lookup.end()) {
		return lookup.rbegin()->second;
	} else if (upper_it == lookup.begin()) {
		return upper_it->second;
	} else {
		// Linearly interpolate between the greater and lower points
		const auto lower_it = std::prev(upper_it);
		const auto brightness_range = (upper_it->first - lower_it->first);
		const auto blend_amount = ((brightness - lower_it->first) / brightness_range);
		return std::lerp(lower_it->second, upper_it->second, blend_amount);
	}

	return brightness;
}
} // namespace

Property::Property(vr::PropertyTypeTag_t tag, void *buffer, uint32_t bufferSize)
{
	this->tag = tag;
	this->buffer.resize(bufferSize);
	std::memcpy(this->buffer.data(), buffer, bufferSize);
}

HmdDevice::HmdDevice(const DeviceBuilder &builder) : Device(builder)
{
	this->name = XRT_DEVICE_GENERIC_HMD;
	this->device_type = XRT_DEVICE_TYPE_HMD;
	this->container_handle = 0;

	inputs_vec = {xrt_input{true, 0, XRT_INPUT_GENERIC_HEAD_POSE, {}}};
	this->inputs = inputs_vec.data();
	this->input_count = inputs_vec.size();

#define SETUP_MEMBER_FUNC(name) this->xrt_device::name = &device_bouncer<HmdDevice, &HmdDevice::name, xrt_result_t>
	SETUP_MEMBER_FUNC(get_view_poses);
	SETUP_MEMBER_FUNC(compute_distortion);
	SETUP_MEMBER_FUNC(set_brightness);
	SETUP_MEMBER_FUNC(get_brightness);
#undef SETUP_MEMBER_FUNC
}

ControllerDevice::ControllerDevice(vr::PropertyContainerHandle_t handle, const DeviceBuilder &builder) : Device(builder)
{
	this->device_type = XRT_DEVICE_TYPE_UNKNOWN;
	this->container_handle = handle;

	this->xrt_device::get_hand_tracking =
	    &device_bouncer<ControllerDevice, &ControllerDevice::get_hand_tracking, xrt_result_t>;
	this->xrt_device::set_output = &device_bouncer<ControllerDevice, &ControllerDevice::set_output, xrt_result_t>;
}

Device::~Device()
{
	m_relation_history_destroy(&relation_hist);
}

Device::Device(const DeviceBuilder &builder) : xrt_device({}), ctx(builder.ctx), driver(builder.driver)
{
	m_relation_history_create(&relation_hist);
	std::strncpy(this->serial, builder.serial, XRT_DEVICE_NAME_LEN - 1);
	this->serial[XRT_DEVICE_NAME_LEN - 1] = 0;
	this->tracking_origin = ctx.get();
	this->supported.orientation_tracking = true;
	this->supported.position_tracking = true;
	this->supported.hand_tracking = true;
	this->supported.force_feedback = false;
	this->supported.form_factor_check = false;
	this->supported.battery_status = true;
	this->supported.brightness_control = true;

	this->xrt_device::update_inputs = &device_bouncer<Device, &Device::update_inputs, xrt_result_t>;
#define SETUP_MEMBER_FUNC(name) this->xrt_device::name = &device_bouncer<Device, &Device::name>
	SETUP_MEMBER_FUNC(get_tracked_pose);
	SETUP_MEMBER_FUNC(get_battery_status);
#undef SETUP_MEMBER_FUNC

	this->xrt_device::destroy = [](xrt_device *xdev) {
		auto *dev = static_cast<Device *>(xdev);
		dev->driver->Deactivate();
		delete dev;
	};

	init_chaperone(builder.steam_install);
}

void
ControllerDevice::set_hand_tracking_hand(xrt_input_name name)
{
	if (has_index_hand_tracking) {
		inputs_map["HAND"]->name = name;
	}
}

// NOTE: No operations that would force inputs_vec or finger_inputs_vec to reallocate (such as insertion)
// should be done after this function is called, otherwise the pointers in inputs_map/finger_inputs_map
// would be invalidated.
void
ControllerDevice::set_input_class(const InputClass *input_class)
{
	// this should only be called once
	assert(inputs_vec.empty());
	this->input_class = input_class;

	// reserve to ensure our pointers don't get invalidated
	inputs_vec.reserve(input_class->poses.size() + input_class->non_poses.size() + 1);
	for (xrt_input_name input : input_class->poses) {
		inputs_vec.push_back({true, 0, input, {}});
	}
	for (const auto &[path, input] : input_class->non_poses) {
		assert(inputs_vec.capacity() >= inputs_vec.size() + 1);
		inputs_vec.push_back({true, 0, input, {}});
		inputs_map.insert({path, &inputs_vec.back()});
	}

	has_index_hand_tracking = debug_get_bool_option_lh_emulate_hand() && !input_class->finger_curls.empty();
	if (has_index_hand_tracking) {
		finger_inputs_vec.reserve(input_class->finger_curls.size());
		for (const auto &[path, finger] : input_class->finger_curls) {
			assert(finger_inputs_vec.capacity() >= finger_inputs_vec.size() + 1);
			finger_inputs_vec.push_back({0, finger, 0.f});
			finger_inputs_map.insert({path, &finger_inputs_vec.back()});
		}
		assert(inputs_vec.capacity() >= inputs_vec.size() + 1);
		inputs_vec.push_back({true, 0, XRT_INPUT_HT_CONFORMING_LEFT, {}});
		inputs_map.insert({std::string_view("HAND"), &inputs_vec.back()});
	}

	this->inputs = inputs_vec.data();
	this->input_count = inputs_vec.size();
}

xrt_hand
ControllerDevice::get_xrt_hand()
{
	switch (this->device_type) {
	case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: {
		return xrt_hand::XRT_HAND_LEFT;
	}
	case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: {
		return xrt_hand::XRT_HAND_RIGHT;
	}
	default: DEV_ERR("Device %s cannot be tracked as a hand!", serial); return xrt_hand::XRT_HAND_LEFT;
	}
}

const std::vector<std::string> FACE_BUTTONS = {
    "/input/system/touch", "/input/a/touch", "/input/b/touch", "/input/thumbstick/touch", "/input/trackpad/touch",
};

void
ControllerDevice::update_hand_tracking(int64_t desired_timestamp_ns, struct xrt_hand_joint_set *out)
{
	if (!has_index_hand_tracking)
		return;
	float index = 0.f;
	float middle = 0.f;
	float ring = 0.f;
	float pinky = 0.f;
	float thumb = 0.f;
	for (auto fi : finger_inputs_vec) {
		switch (fi.finger) {
		case IndexFinger::Index: index = fi.value; break;
		case IndexFinger::Middle: middle = fi.value; break;
		case IndexFinger::Ring: ring = fi.value; break;
		case IndexFinger::Pinky: pinky = fi.value; break;
		default: break;
		}
	}
	for (const auto &name : FACE_BUTTONS) {
		auto *input = get_input_from_name(name);
		if (input && input->value.boolean) {
			thumb = 1.f;
			break;
		}
	}
	auto curl_values = u_hand_tracking_curl_values{pinky, ring, middle, index, thumb};

	struct xrt_space_relation hand_relation = {};
	m_relation_history_get(relation_hist, desired_timestamp_ns, &hand_relation);

	u_hand_sim_simulate_for_valve_index_knuckles(&curl_values, get_xrt_hand(), &hand_relation, out);

	struct xrt_relation_chain chain = {};

	struct xrt_pose pose_offset = XRT_POSE_IDENTITY;
	vive_poses_get_pose_offset(name, device_type, inputs_map["HAND"]->name, &pose_offset);

	m_relation_chain_push_pose(&chain, &pose_offset);
	m_relation_chain_push_relation(&chain, &hand_relation);
	m_relation_chain_resolve(&chain, &out->hand_pose);
}

xrt_input *
Device::get_input_from_name(const std::string_view name)
{
	// Return nullptr without any other output to suppress a pile of useless warnings found below.
	if (name == "/input/finger/index" || name == "/input/finger/middle" || name == "/input/finger/ring" ||
	    name == "/input/finger/pinky") {
		return nullptr;
	}
	auto input = inputs_map.find(name);
	if (input == inputs_map.end()) {
		DEV_WARN("requested unknown input name %s for device %s", std::string(name).c_str(), serial);
		return nullptr;
	}
	return input->second;
}

void
ControllerDevice::set_haptic_handle(vr::VRInputComponentHandle_t handle)
{
	// this should only be set once
	assert(output == nullptr);
	DEV_DEBUG("setting haptic handle for %" PRIu64, handle);
	haptic_handle = handle;
	xrt_output_name name;
	switch (this->name) {
	case XRT_DEVICE_VIVE_WAND: {
		name = XRT_OUTPUT_NAME_VIVE_HAPTIC;
		break;
	}
	case XRT_DEVICE_INDEX_CONTROLLER: {
		name = XRT_OUTPUT_NAME_INDEX_HAPTIC;
		break;
	}
	case XRT_DEVICE_VIVE_TRACKER: {
		name = XRT_OUTPUT_NAME_VIVE_TRACKER_HAPTIC;
		break;
	}
	default: {
		DEV_WARN("Unknown device name (%u), haptics will not work", this->name);
		return;
	}
	}
	output = std::make_unique<xrt_output>(xrt_output{name});
	this->output_count = 1;
	this->outputs = output.get();
}

xrt_result_t
Device::update_inputs()
{
	std::lock_guard<std::mutex> lock(frame_mutex);
	ctx->maybe_run_frame(++current_frame);
	return XRT_SUCCESS;
}

IndexFingerInput *
ControllerDevice::get_finger_from_name(const std::string_view name)
{
	auto finger = finger_inputs_map.find(name);
	if (finger == finger_inputs_map.end()) {
		DEV_WARN("requested unknown finger name %s for device %s", std::string(name).c_str(), serial);
		return nullptr;
	}
	return finger->second;
}

xrt_result_t
ControllerDevice::get_hand_tracking(enum xrt_input_name name,
                                    int64_t desired_timestamp_ns,
                                    struct xrt_hand_joint_set *out_value,
                                    int64_t *out_timestamp_ns)
{
	if (!has_index_hand_tracking)
		return XRT_ERROR_NOT_IMPLEMENTED;
	update_hand_tracking(desired_timestamp_ns, out_value);
	out_value->is_active = true;
	hand_tracking_timestamp = desired_timestamp_ns;
	*out_timestamp_ns = hand_tracking_timestamp;
	return XRT_SUCCESS;
}

void
Device::get_pose(uint64_t at_timestamp_ns, xrt_space_relation *out_relation)
{
	m_relation_history_get(this->relation_hist, at_timestamp_ns, out_relation);
}

xrt_result_t
Device::get_battery_status(bool *out_present, bool *out_charging, float *out_charge)
{
	*out_present = this->provides_battery_status;
	*out_charging = this->charging;
	*out_charge = this->charge;
	return XRT_SUCCESS;
}

xrt_result_t
HmdDevice::get_brightness(float *out_brightness)
{
	*out_brightness = this->brightness;
	return XRT_SUCCESS;
}

xrt_result_t
HmdDevice::set_brightness(float brightness, bool relative)
{
	constexpr auto min_brightness = 0.2f;
	constexpr auto max_brightness = 1.5f;

	const auto target_brightness = relative ? (this->brightness + brightness) : brightness;
	this->brightness = std::clamp(target_brightness, min_brightness, max_brightness);
	const auto analog_gain =
	    std::clamp(brightness_to_analog_gain(this->brightness), analog_gain_range.min, analog_gain_range.max);
	ctx->settings.SetFloat(vr::k_pch_SteamVR_Section, analog_gain_settings_key, analog_gain);
	return XRT_SUCCESS;
}

xrt_result_t
HmdDevice::get_tracked_pose(xrt_input_name name, uint64_t at_timestamp_ns, xrt_space_relation *out_relation)
{
	switch (name) {
	case XRT_INPUT_GENERIC_HEAD_POSE: Device::get_pose(at_timestamp_ns, out_relation); break;
	default: U_LOG_XDEV_UNSUPPORTED_INPUT(this, ctx->log_level, name); return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	return XRT_SUCCESS;
}

xrt_result_t
ControllerDevice::get_tracked_pose(xrt_input_name name, uint64_t at_timestamp_ns, xrt_space_relation *out_relation)
{
	xrt_space_relation rel = {};
	Device::get_pose(at_timestamp_ns, &rel);

	xrt_pose pose_offset = XRT_POSE_IDENTITY;
	vive_poses_get_pose_offset(input_class->name, device_type, name, &pose_offset);

	xrt_relation_chain relchain = {};

	m_relation_chain_push_pose(&relchain, &pose_offset);
	m_relation_chain_push_relation(&relchain, &rel);
	m_relation_chain_resolve(&relchain, out_relation);

	struct xrt_pose *p = &out_relation->pose;
	DEV_DEBUG("controller %u: GET_POSITION (%f %f %f) GET_ORIENTATION (%f, %f, %f, %f)", name, p->position.x,
	          p->position.y, p->position.z, p->orientation.x, p->orientation.y, p->orientation.z, p->orientation.w);

	return XRT_SUCCESS;
}

xrt_result_t
ControllerDevice::set_output(xrt_output_name name, const xrt_output_value *value)

{
	const auto &vib = value->vibration;
	if (vib.amplitude == 0.0)
		return XRT_SUCCESS;
	vr::VREvent_HapticVibration_t event;
	event.containerHandle = container_handle;
	event.componentHandle = haptic_handle;
	event.fDurationSeconds = (float)vib.duration_ns / 1e9f;
	// 0.0f in OpenXR means let the driver determine a frequency, but
	// in OpenVR means no haptic, so let's set a reasonable default.
	float frequency = vib.frequency;
	if (frequency == 0.0) {
		frequency = 200.0f;
	}

	event.fFrequency = frequency;
	event.fAmplitude = vib.amplitude;

	ctx->add_haptic_event(event);
	return XRT_SUCCESS;
}

void
HmdDevice::SetDisplayEyeToHead(uint32_t unWhichDevice,
                               const vr::HmdMatrix34_t &eyeToHeadLeft,
                               const vr::HmdMatrix34_t &eyeToHeadRight)
{
	xrt_matrix_3x3 leftEye_prequat;
	xrt_matrix_3x3 rightEye_prequat;

	xrt_pose leftEye_postquat;
	xrt_pose rightEye_postquat;

	// This is a HmdMatrix34 to xrt_matrix_3x3 copy.
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			leftEye_prequat.v[i * 3 + j] = eyeToHeadLeft.m[i][j];
			rightEye_prequat.v[i * 3 + j] = eyeToHeadRight.m[i][j];
		}
	}

	math_quat_from_matrix_3x3(&leftEye_prequat, &leftEye_postquat.orientation);
	math_quat_from_matrix_3x3(&rightEye_prequat, &rightEye_postquat.orientation);
	leftEye_postquat.position.x = eyeToHeadLeft.m[0][3];
	leftEye_postquat.position.y = eyeToHeadLeft.m[1][3];
	leftEye_postquat.position.z = eyeToHeadLeft.m[2][3];

	rightEye_postquat.position.x = eyeToHeadRight.m[0][3];
	rightEye_postquat.position.y = eyeToHeadRight.m[1][3];
	rightEye_postquat.position.z = eyeToHeadRight.m[2][3];

	this->eye[0].orientation = leftEye_postquat.orientation;
	this->eye[0].position = leftEye_postquat.position;
	this->eye[1].orientation = rightEye_postquat.orientation;
	this->eye[1].position = rightEye_postquat.position;
}

xrt_result_t
HmdDevice::get_view_poses(const xrt_vec3 *default_eye_relation,
                          uint64_t at_timestamp_ns,
                          uint32_t view_count,
                          xrt_space_relation *out_head_relation,
                          xrt_fov *out_fovs,
                          xrt_pose *out_poses)
{
	struct xrt_vec3 eye_relation = *default_eye_relation;
	eye_relation.x = ipd;

	xrt_result_t xret = u_device_get_view_poses( //
	    this,                                    //
	    &eye_relation,                           //
	    at_timestamp_ns,                         //
	    view_count,                              //
	    out_head_relation,                       //
	    out_fovs,                                //
	    out_poses);                              //
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	out_poses[0].orientation = this->eye[0].orientation;
	out_poses[0].position.z = this->eye[0].position.z;
	out_poses[0].position.y = this->eye[0].position.y;
	out_poses[1].orientation = this->eye[1].orientation;
	out_poses[1].position.z = this->eye[1].position.z;
	out_poses[1].position.y = this->eye[1].position.y;

	return XRT_SUCCESS;
}

xrt_result_t
HmdDevice::compute_distortion(uint32_t view, float u, float v, xrt_uv_triplet *out_result)
{
	// Vive Pro 2 has a vertically flipped distortion map.
	if (this->variant == VIVE_VARIANT_PRO2) {
		v = 1.0f - v;
	}

	vr::EVREye eye = (view == 0) ? vr::Eye_Left : vr::Eye_Right;
	vr::DistortionCoordinates_t coords = this->hmd_parts->display->ComputeDistortion(eye, u, v);
	out_result->r = {coords.rfRed[0], coords.rfRed[1]};
	out_result->g = {coords.rfGreen[0], coords.rfGreen[1]};
	out_result->b = {coords.rfBlue[0], coords.rfBlue[1]};
	return XRT_SUCCESS;
}

void
HmdDevice::set_hmd_parts(std::unique_ptr<Parts> parts)
{
	{
		std::lock_guard lk(hmd_parts_mut);
		hmd_parts = std::move(parts);
		this->hmd = &hmd_parts->base;
	}
	hmd_parts_cv.notify_all();
}

namespace {
xrt_quat
copy_quat(const vr::HmdQuaternion_t &quat)
{
	return xrt_quat{(float)quat.x, (float)quat.y, (float)quat.z, (float)quat.w};
}

xrt_vec3
copy_vec3(const double (&vec)[3])
{
	return xrt_vec3{(float)vec[0], (float)vec[1], (float)vec[2]};
}

xrt_pose
copy_pose(const vr::HmdQuaternion_t &orientation, const double (&position)[3])
{
	return xrt_pose{copy_quat(orientation), copy_vec3(position)};
}
} // namespace

void
Device::init_chaperone(const std::string &steam_install)
{
	static bool initialized = false;
	if (initialized)
		return;

	initialized = true;

	// Lighthouse driver seems to create a lighthousedb.json and a chaperone_info.vrchap (which is json)
	// We will use the known_universes from the lighthousedb.json to match to a universe from chaperone_info.vrchap

	using xrt::auxiliary::util::json::JSONNode;
	auto lighthousedb = JSONNode::loadFromFile(steam_install + "/config/lighthouse/lighthousedb.json");
	if (lighthousedb.isInvalid()) {
		DEV_ERR("Couldn't load lighthousedb file, playspace center will be off - was Room Setup run?");
		return;
	}
	auto chap_info = JSONNode::loadFromFile(steam_install + "/config/chaperone_info.vrchap");
	if (chap_info.isInvalid()) {
		DEV_ERR("Couldn't load chaperone info, playspace center will be off - was Room Setup run?");
		return;
	}

	JSONNode info = {};
	bool universe_found = false;

	// XXX: This may be broken if there are multiple known universes - how do we determine which to use then?
	auto known_universes = lighthousedb["known_universes"].asArray();
	for (auto &universe : known_universes) {
		const std::string id = universe["id"].asString();
		for (const JSONNode &u : chap_info["universes"].asArray()) {
			if (u["universeID"].asString() == id) {
				DEV_INFO("Found info for universe %s", id.c_str());
				info = u;
				universe_found = true;
				break;
			}
		}
		if (universe_found) {
			break;
		}
	}

	if (info.isInvalid()) {
		DEV_ERR("Couldn't find chaperone info for any known universe, playspace center will be off");
		return;
	}

	std::vector<JSONNode> translation_arr = info["standing"]["translation"].asArray();

	// If the array is missing elements, add zero.
	for (size_t i = translation_arr.size(); i < 3; i++) {
		translation_arr.push_back(JSONNode("0.0"));
	}

	const double yaw = info["standing"]["yaw"].asDouble();
	const xrt_vec3 yaw_axis{0.0, -1.0, 0.0};
	math_quat_from_angle_vector(static_cast<float>(yaw), &yaw_axis, &chaperone.orientation);
	chaperone.position = copy_vec3({
	    translation_arr[0].asDouble(),
	    translation_arr[1].asDouble(),
	    translation_arr[2].asDouble(),
	});
	math_quat_rotate_vec3(&chaperone.orientation, &chaperone.position, &chaperone.position);
	DEV_INFO("Initialized chaperone data.");
}

inline xrt_space_relation_flags
operator|(xrt_space_relation_flags a, xrt_space_relation_flags b)
{
	return static_cast<xrt_space_relation_flags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline xrt_space_relation_flags &
operator|=(xrt_space_relation_flags &a, xrt_space_relation_flags b)
{
	a = a | b;
	return a;
}

void
Device::update_pose(const vr::DriverPose_t &newPose) const
{
	xrt_space_relation relation = {};

	if (newPose.poseIsValid) {
		// The pose is known to be valid but that alone is not enough to say whether the data comes from
		// inference or if it represents actively tracked position and orientation data. Furthermore we avoid
		// assumptions regarding the validity of time-derivatives until we know that they're based on tracked
		// data. This is a conservative strategy that should reduce concerns regarding drift. see
		// https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrSpaceLocationFlagBits.html
		relation.relation_flags |= xrt_space_relation_flags::XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		                           xrt_space_relation_flags::XRT_SPACE_RELATION_POSITION_VALID_BIT;

		switch (newPose.result) {
		// see https://github.com/ValveSoftware/openvr/blob/master/docs/Driver_API_Documentation.md
		case vr::TrackingResult_Running_OK:
			// If the tracker is running ok then we have actively tracked 6DoF data
			relation.relation_flags |=
			    xrt_space_relation_flags::XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
			    xrt_space_relation_flags::XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
			    xrt_space_relation_flags::XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT |
			    xrt_space_relation_flags::XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
			break;
		case vr::TrackingResult_Fallback_RotationOnly:
		case vr::TrackingResult_Running_OutOfRange:
			// If the tracking is degraded we should still be able to assume that we still have tracked 3DoF
			// data
			relation.relation_flags |=
			    xrt_space_relation_flags::XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
			    xrt_space_relation_flags::XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
			break;
		default: break;
		}
	}

	// The driver still outputs good pose data regardless of the pose results above
	relation.pose = copy_pose(newPose.qRotation, newPose.vecPosition);
	relation.linear_velocity = copy_vec3(newPose.vecVelocity);
	relation.angular_velocity = copy_vec3(newPose.vecAngularVelocity);

	math_quat_rotate_vec3(&relation.pose.orientation, &relation.angular_velocity, &relation.angular_velocity);

	// apply over local transform
	const xrt_pose local = copy_pose(newPose.qDriverFromHeadRotation, newPose.vecDriverFromHeadTranslation);
	math_pose_transform(&relation.pose, &local, &relation.pose);

	// apply world transform
	const xrt_pose world = copy_pose(newPose.qWorldFromDriverRotation, newPose.vecWorldFromDriverTranslation);
	math_pose_transform(&world, &relation.pose, &relation.pose);
	math_quat_rotate_vec3(&world.orientation, &relation.linear_velocity, &relation.linear_velocity);
	math_quat_rotate_vec3(&world.orientation, &relation.angular_velocity, &relation.angular_velocity);

	// apply chaperone transform
	math_pose_transform(&chaperone, &relation.pose, &relation.pose);
	math_quat_rotate_vec3(&chaperone.orientation, &relation.linear_velocity, &relation.linear_velocity);
	math_quat_rotate_vec3(&chaperone.orientation, &relation.angular_velocity, &relation.angular_velocity);

	const uint64_t ts = chrono_timestamp_ns() + static_cast<uint64_t>(newPose.poseTimeOffset * 1000000.0);

	m_relation_history_push(relation_hist, &relation, ts);
}

vr::ETrackedPropertyError
Device::handle_properties(const vr::PropertyWrite_t *batch, uint32_t count)
{
	for (uint32_t i = 0; i < count; ++i) {
		vr::ETrackedPropertyError err = handle_property_write(batch[i]);
		if (err != vr::ETrackedPropertyError::TrackedProp_Success) {
			return err;
		}
	}
	return vr::ETrackedPropertyError::TrackedProp_Success;
}

vr::ETrackedPropertyError
Device::handle_read_properties(vr::PropertyRead_t *batch, uint32_t count)
{
	for (uint32_t i = 0; i < count; ++i) {
		vr::ETrackedPropertyError err = handle_generic_property_read(batch[i]);
		if (err != vr::ETrackedPropertyError::TrackedProp_Success) {
			return err;
		}
	}
	return vr::ETrackedPropertyError::TrackedProp_Success;
}

void
HmdDevice::set_nominal_frame_interval(uint64_t interval_ns)
{
	auto set = [this, interval_ns] { hmd_parts->base.screens[0].nominal_frame_interval_ns = interval_ns; };

	if (hmd_parts) {
		set();
	} else {
		std::thread t([this, set] {
			std::unique_lock lk(hmd_parts_mut);
			hmd_parts_cv.wait(lk, [this] { return hmd_parts != nullptr; });
			set();
		});
		t.detach();
	}
}

namespace {
// From openvr driver documentation
// (https://github.com/ValveSoftware/openvr/blob/master/docs/Driver_API_Documentation.md#Input-Profiles):
// "Input profiles are expected to be a valid JSON file,
// and should be located: <driver_name>/resources/input/<device_name>_profile.json"
// So we will just parse the file name to get the device name.
std::string_view
parse_profile(std::string_view path)
{
	size_t name_start_idx = path.find_last_of('/') + 1;
	size_t name_end_idx = path.find_last_of('_');
	return path.substr(name_start_idx, name_end_idx - name_start_idx);
}
} // namespace

vr::ETrackedPropertyError
Device::handle_generic_property_write(const vr::PropertyWrite_t &prop)
{
	switch (prop.writeType) {
	case vr::EPropertyWriteType::PropertyWrite_Set:
		if (properties.count(prop.prop) > 0) {
			Property &p = properties.at(prop.prop);
			if (p.tag != prop.unTag) {
				return vr::ETrackedPropertyError::TrackedProp_WrongDataType;
			}
			p.buffer.resize(prop.unBufferSize);
			std::memcpy(p.buffer.data(), prop.pvBuffer, prop.unBufferSize);
			return vr::ETrackedPropertyError::TrackedProp_Success;
		} else {
			properties.emplace(std::piecewise_construct, std::forward_as_tuple(prop.prop),
			                   std::forward_as_tuple(prop.unTag, prop.pvBuffer, prop.unBufferSize));
		}
		break;
	case vr::EPropertyWriteType::PropertyWrite_Erase: properties.erase(prop.prop); break;
	case vr::EPropertyWriteType::PropertyWrite_SetError:
		DEV_DEBUG("Property write type SetError not supported! (property %d)", prop.prop);
		break;
	}
	return vr::ETrackedPropertyError::TrackedProp_Success;
}

vr::ETrackedPropertyError
Device::handle_generic_property_read(vr::PropertyRead_t &prop)
{
	if (properties.count(prop.prop) == 0) {
		// not verified if this is the correct error
		return vr::ETrackedPropertyError::TrackedProp_UnknownProperty;
	}
	Property &p = properties.at(prop.prop);
	prop.unTag = p.tag;
	prop.unRequiredBufferSize = p.buffer.size();
	if (prop.pvBuffer == nullptr || prop.unBufferSize < p.buffer.size()) {
		prop.eError = vr::ETrackedPropertyError::TrackedProp_BufferTooSmall;
		return prop.eError;
	}
	std::memcpy(prop.pvBuffer, p.buffer.data(), p.buffer.size());
	prop.eError = vr::ETrackedPropertyError::TrackedProp_Success;
	return prop.eError;
}

vr::ETrackedPropertyError
Device::handle_property_write(const vr::PropertyWrite_t &prop)
{
	switch (prop.prop) {
	case vr::Prop_ManufacturerName_String: {
		this->manufacturer = std::string(static_cast<char *>(prop.pvBuffer), prop.unBufferSize);
		if (!this->model.empty()) {
			std::snprintf(this->str, std::size(this->str), "%s %s", this->manufacturer.c_str(),
			              this->model.c_str());
		}
		break;
	}
	case vr::Prop_ModelNumber_String: {
		this->model = std::string(static_cast<char *>(prop.pvBuffer), prop.unBufferSize);
		if (!this->manufacturer.empty()) {
			std::snprintf(this->str, std::size(this->str), "%s %s", this->manufacturer.c_str(),
			              this->model.c_str());
		}
		break;
	}
	default: {
		DEV_DEBUG("Unhandled property: %i", prop.prop);
		break;
	}
	}
	return handle_generic_property_write(prop);
}

bool
HmdDevice::init_vive_pro_2(struct xrt_prober *xp)
{
	xrt_result_t xret;
	int ret = 0;

	struct xrt_prober_device **devices = nullptr;
	size_t device_count;

	xret = xrt_prober_lock_list(xp, &devices, &device_count);
	if (xret != XRT_SUCCESS) {
		DEV_ERR("Failed to lock prober device list");
		return false;
	}

	for (size_t i = 0; i < device_count; i++) {
		struct xrt_prober_device *dev = devices[i];

		if (dev->vendor_id == VP2_VID && dev->product_id == VP2_PID) {
			DEV_INFO("Found Vive Pro 2 HID device");
			struct os_hid_device *hid_dev = nullptr;
			ret = xrt_prober_open_hid_interface(xp, dev, 0, &hid_dev);
			if (ret != 0) {
				DEV_ERR("Failed to open Vive Pro 2 HID interface");
				break;
			}

			ret = vp2_hid_open(hid_dev, &this->vp2.hid);
			if (ret != 0) {
				DEV_ERR("Failed to open Vive Pro 2 HID device");
				break;
			}

			break;
		}
	}

	xrt_prober_unlock_list(xp, &devices);

	int width, height;
	vp2_resolution_get_extents(vp2_get_resolution(this->vp2.hid), &width, &height);

	for (int i = 0; i < 2; i++) {
		this->hmd_parts->base.views[i].display.w_pixels = width / 2;
		this->hmd_parts->base.views[i].display.h_pixels = height;

		this->hmd_parts->base.views[i].viewport.w_pixels = width / 2;
		this->hmd_parts->base.views[i].viewport.h_pixels = height;
	}

	this->hmd_parts->base.views[1].viewport.x_pixels = width / 2;

	this->hmd_parts->base.screens[0].w_pixels = width;
	this->hmd_parts->base.screens[0].h_pixels = height;

	return ret == 0;
}

vr::ETrackedPropertyError
HmdDevice::handle_property_write(const vr::PropertyWrite_t &prop)
{
	switch (prop.prop) {
	case vr::Prop_ModelNumber_String: {
		std::string model_number(static_cast<char *>(prop.pvBuffer), prop.unBufferSize);

		this->variant = vive_determine_variant(model_number.c_str());

		return Device::handle_property_write(prop);
	}
	case vr::Prop_DisplayFrequency_Float: {
		assert(prop.unBufferSize == sizeof(float));
		float freq = *static_cast<float *>(prop.pvBuffer);
		set_nominal_frame_interval((1.f / freq) * 1e9f);
		break;
	}
	case vr::Prop_UserIpdMeters_Float: {
		if (*static_cast<float *>(prop.pvBuffer) != 0) {
			ipd = *static_cast<float *>(prop.pvBuffer);
		}
		break;
	}
	case vr::Prop_SecondsFromVsyncToPhotons_Float: {
		vsync_to_photon_ns = *static_cast<float *>(prop.pvBuffer) * 1e9f;
		break;
	}
	case vr::Prop_DeviceProvidesBatteryStatus_Bool: {
		float supported = *static_cast<bool *>(prop.pvBuffer);
		this->provides_battery_status = supported;
		DEV_DEBUG("Has battery status: HMD: %s", supported ? "true" : "false");
		break;
	}
	case vr::Prop_DeviceIsCharging_Bool: {
		float charging = *static_cast<bool *>(prop.pvBuffer);
		this->charging = charging;
		DEV_DEBUG("Charging: HMD: %s", charging ? "true" : "false");
		break;
	}
	case vr::Prop_DeviceBatteryPercentage_Float: {
		float bat = *static_cast<float *>(prop.pvBuffer);
		this->charge = bat;
		DEV_DEBUG("Battery: HMD: %f", bat);
		break;
	}
	case vr::Prop_DisplaySupportsAnalogGain_Bool: {
		this->supported.brightness_control = *static_cast<bool *>(prop.pvBuffer);
		break;
	}
	case vr::Prop_DisplayMinAnalogGain_Float: {
		this->analog_gain_range.min = *static_cast<float *>(prop.pvBuffer);
		break;
	}
	case vr::Prop_DisplayMaxAnalogGain_Float: {
		this->analog_gain_range.max = *static_cast<float *>(prop.pvBuffer);
		break;
	}
	default: {
		return Device::handle_property_write(prop);
	}
	}
	return handle_generic_property_write(prop);
}

vr::ETrackedPropertyError
ControllerDevice::handle_property_write(const vr::PropertyWrite_t &prop)
{
	switch (prop.prop) {
	case vr::Prop_InputProfilePath_String: {
		std::string_view profile =
		    parse_profile(std::string_view(static_cast<char *>(prop.pvBuffer), prop.unBufferSize));
		auto input_class = controller_classes.find(profile);
		if (input_class == controller_classes.end()) {
			DEV_ERR("Could not find input class for controller profile %s", std::string(profile).c_str());
		} else {
			this->name = input_class->second.name;
			set_input_class(&input_class->second);
		}
		break;
	}
	case vr::Prop_ModelNumber_String: {
		using namespace std::literals::string_view_literals;
		vr::PropertyWrite_t fixedProp = prop;
		const std::string_view name = {static_cast<char *>(prop.pvBuffer), prop.unBufferSize};
		if (name == "SlimeVR Virtual Tracker\0"sv) {
			static const InputClass input_class = {
			    XRT_DEVICE_VIVE_TRACKER, {XRT_INPUT_GENERIC_TRACKER_POSE}, {}, {}};
			this->name = input_class.name;
			set_input_class(&input_class);
			this->manufacturer = name.substr(0, name.find_first_of(' '));
			fixedProp.pvBuffer = (char *)fixedProp.pvBuffer + this->manufacturer.size() +
			                     (this->manufacturer.size() != name.size());
			fixedProp.unBufferSize = name.end() - (char *)fixedProp.pvBuffer;
		}
		return Device::handle_property_write(fixedProp);
	}
	case vr::Prop_ControllerRoleHint_Int32: {
		vr::ETrackedControllerRole role = *static_cast<vr::ETrackedControllerRole *>(prop.pvBuffer);
		switch (role) {
		case vr::TrackedControllerRole_Invalid: {
			this->device_type = XRT_DEVICE_TYPE_ANY_HAND_CONTROLLER;
			break;
		}
		case vr::TrackedControllerRole_RightHand: {
			this->device_type = XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
			set_hand_tracking_hand(XRT_INPUT_HT_CONFORMING_RIGHT);
			break;
		}
		case vr::TrackedControllerRole_LeftHand: {
			this->device_type = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
			set_hand_tracking_hand(XRT_INPUT_HT_CONFORMING_LEFT);
			break;
		}
		case vr::TrackedControllerRole_OptOut: {
			this->device_type = XRT_DEVICE_TYPE_GENERIC_TRACKER;
			break;
		}
		default: {
			this->device_type = XRT_DEVICE_TYPE_UNKNOWN;
			DEV_WARN("requested unimplemented role hint %i", this->device_type);
			break;
		}
		}
		break;
	}
	case vr::Prop_DeviceProvidesBatteryStatus_Bool: {
		float supported = *static_cast<bool *>(prop.pvBuffer);
		const char *name;
		switch (this->device_type) {
		case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: {
			name = "Left";
			break;
		}
		case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: {
			name = "Right";
			break;
		}
		default: {
			name = "Unknown";
			break;
		}
		}
		this->provides_battery_status = supported;
		DEV_DEBUG("Has battery status: %s: %s", name, supported ? "true" : "false");
		break;
	}
	case vr::Prop_DeviceIsCharging_Bool: {
		float charging = *static_cast<bool *>(prop.pvBuffer);
		const char *name;
		switch (this->device_type) {
		case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: {
			name = "Left";
			break;
		}
		case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: {
			name = "Right";
			break;
		}
		default: {
			name = "Unknown";
		}
		}
		this->charging = charging;
		DEV_DEBUG("Charging: %s: %s", name, charging ? "true" : "false");
		break;
	}
	case vr::Prop_DeviceBatteryPercentage_Float: {
		float bat = *static_cast<float *>(prop.pvBuffer);
		const char *name;
		switch (this->device_type) {
		case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: {
			name = "Left";
			break;
		}
		case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: {
			name = "Right";
			break;
		}
		default: {
			name = "Unknown";
		}
		}
		this->charge = bat;
		DEV_DEBUG("Battery: %s: %f", name, bat);
		break;
	}
	default: {
		return Device::handle_property_write(prop);
	}
	}
	return handle_generic_property_write(prop);
}
