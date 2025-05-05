// Copyright 2020-2021, Collabora, Ltd.
// Copyright 2023, Jan Schmidt
// Copyright 2024, Joel Valenciano
// Copyright 2025, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR2 HMD device
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @author Joel Valenciano <joelv1907@gmail.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_psvr2
 */

#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_tracking.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_clock_tracking.h"
#include "math/m_mathinclude.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_frame.h"
#include "util/u_logging.h"
#include "util/u_sink.h"
#include "util/u_time.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#include <stdio.h>
#include <assert.h>
#include <inttypes.h>
#include <libusb.h>

#include "psvr2.h"


/*
 *
 * Structs and defines.
 *
 */
#define USB_SLAM_XFER_SIZE 1024

#define USB_STATUS_XFER_SIZE 1024

#define USB_CAM_MODE10_XFER_SIZE 1040640
#define USB_CAM_MODE1_XFER_SIZE 819456
#define NUM_CAM_XFERS 1

#define USB_LD_XFER_SIZE 36944

#define USB_RP_XFER_SIZE 821120

#define USB_VD_XFER_SIZE 32768

#define GYRO_SCALE (2000.0 / 32767.0)
#define ACCEL_SCALE (4.0 * MATH_GRAVITY_M_S2 / 32767.0)

#define DEG_TO_RAD(D) ((D)*M_PI / 180.)

#define SLAM_POSE_CORRECTION                                                                                           \
	{                                                                                                              \
		.orientation = {.x = 0, .y = 0, .z = sqrt(2) / 2, .w = sqrt(2) / 2 }                                   \
	}

/*!
 * PSVR2 HMD device
 *
 * @implements xrt_device
 */
struct psvr2_hmd
{
	struct xrt_device base;

	struct xrt_pose pose;

	enum u_logging_level log_level;

	struct os_mutex data_lock;

	uint8_t dprx_status;
	bool proximity_sensor;
	bool passthrough_button;

	bool ipd_updated;
	uint8_t ipd_mm;

	bool camera_enable;
	enum psvr2_camera_mode camera_mode;
	struct u_var_button camera_enable_btn;
	struct u_var_button camera_mode_btn;

	/* IMU input data */
	uint32_t last_vts_us; //< Last VTS timestamp, in microseconds
	uint16_t last_imu_ts;
	struct xrt_vec3 last_gyro;
	struct xrt_vec3 last_accel;

	/* SLAM input data */
	uint32_t last_slam_ts_us; //< Last slam timestamp, in microseconds
	struct xrt_pose last_slam_pose;

	struct xrt_pose slam_correction_pose;
	struct u_var_button slam_correction_set_btn;
	struct u_var_button slam_correction_reset_btn;

	/* Display parameters */
	struct u_device_simple_info info;

	struct u_sink_debug debug_sinks[4];

	/* USB communication */
	libusb_context *ctx;
	libusb_device_handle *dev;

	struct os_thread_helper usb_thread;
	int usb_complete;
	int usb_active_xfers;

	/* Status report */
	struct libusb_transfer *status_xfer;
	/* SLAM (bulk) transfer */
	struct libusb_transfer *slam_xfer;
	/* Camera (bulk) transfers */
	struct libusb_transfer *camera_xfers[NUM_CAM_XFERS];
	/* LD EP9 (bulk) transfer */
	struct libusb_transfer *led_detector_xfer;
	/* RP EP10 (bulk) transfer */
	struct libusb_transfer *relocalizer_xfer;
	/* VD EP11 (bulk) transfer */
	struct libusb_transfer *vd_xfer;

	/* Distortion calibration parameters, to be used with
	 * psvr2_compute_distortion_asymmetric. Very specific to
	 * PS VR2. */
	float distortion_calibration[8];

	bool timestamp_initialized;

	timepoint_ns last_vts_ns;
	timepoint_ns last_slam_ns;
	timepoint_ns system_zero_ns;
	timepoint_ns last_imu_ns;

	time_duration_ns hw2mono;
	time_duration_ns hw2mono_imu;
};


/// Casting helper function
static inline struct psvr2_hmd *
psvr2_hmd(struct xrt_device *xdev)
{
	return (struct psvr2_hmd *)xdev;
}

DEBUG_GET_ONCE_LOG_OPTION(psvr2_log, "PSVR2_LOG", U_LOGGING_WARN)

#define PSVR2_TRACE(p, ...) U_LOG_XDEV_IFL_T(&p->base, p->log_level, __VA_ARGS__)
#define PSVR2_TRACE_HEX(p, data, data_size) U_LOG_XDEV_IFL_T_HEX(&p->base, p->log_level, data, data_size)
#define PSVR2_DEBUG(p, ...) U_LOG_XDEV_IFL_D(&p->base, p->log_level, __VA_ARGS__)
#define PSVR2_DEBUG_HEX(p, data, data_size) U_LOG_XDEV_IFL_D_HEX(&p->base, p->log_level, data, data_size)
#define PSVR2_ERROR(p, ...) U_LOG_XDEV_IFL_E(&p->base, p->log_level, __VA_ARGS__)

static void
psvr2_usb_stop(struct psvr2_hmd *hmd);

static void
psvr2_usb_destroy(struct psvr2_hmd *hmd);

static void
psvr2_hmd_destroy(struct xrt_device *xdev)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	os_thread_helper_lock(&hmd->usb_thread);
	hmd->usb_complete = 1;
	os_thread_helper_unlock(&hmd->usb_thread);
	os_thread_helper_stop_and_wait(&hmd->usb_thread);

	psvr2_usb_destroy(hmd);

	if (hmd->dev != NULL) {
		libusb_close(hmd->dev);
	}

	// Remove the variable tracking.
	u_var_remove_root(hmd);

	os_thread_helper_destroy(&hmd->usb_thread);
	os_mutex_destroy(&hmd->data_lock);
	u_device_free(&hmd->base);
}

static bool
psvr2_compute_distortion(struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *result)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	return psvr2_compute_distortion_asymmetric(hmd->distortion_calibration, result, view, u, v);
}

static xrt_result_t
psvr2_hmd_update_inputs(struct xrt_device *xdev)
{
	// Empty, you should put code to update the attached input fields (if any)
	return XRT_SUCCESS;
}

static xrt_result_t
psvr2_hmd_get_tracked_pose(struct xrt_device *xdev,
                           enum xrt_input_name name,
                           int64_t at_timestamp_ns,
                           struct xrt_space_relation *out_relation)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		PSVR2_ERROR(hmd, "unknown input name");
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	os_mutex_lock(&hmd->data_lock);

	// Estimate pose at timestamp at_timestamp_ns!
	timepoint_ns prediction_ns_mono = at_timestamp_ns - hmd->system_zero_ns;
	timepoint_ns prediction_ns_hw = prediction_ns_mono - hmd->hw2mono;
	(void)prediction_ns_hw; // @todo Use estimated time to predict the headset pose.

	math_quat_normalize(&hmd->pose.orientation);
	out_relation->pose = hmd->pose;
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

	os_mutex_unlock(&hmd->data_lock);

	return XRT_SUCCESS;
}

static void
psvr2_hmd_get_view_poses(struct xrt_device *xdev,
                         const struct xrt_vec3 *default_eye_relation,
                         int64_t at_timestamp_ns,
                         uint32_t view_count,
                         struct xrt_space_relation *out_head_relation,
                         struct xrt_fov *out_fovs,
                         struct xrt_pose *out_poses)
{
	struct psvr2_hmd *hmd = psvr2_hmd(xdev);

	os_mutex_lock(&hmd->data_lock);
	if (hmd->ipd_updated) {
		hmd->info.lens_horizontal_separation_meters = hmd->ipd_mm / 1000.0;
		PSVR2_DEBUG(hmd, "IPD updated to %u mm", hmd->ipd_mm);
		hmd->ipd_updated = false;
	}
	os_mutex_unlock(&hmd->data_lock);

	struct xrt_vec3 eye_relation = *default_eye_relation;
	eye_relation.x = hmd->info.lens_horizontal_separation_meters;

	u_device_get_view_poses(xdev, &eye_relation, at_timestamp_ns, view_count, out_head_relation, out_fovs,
	                        out_poses);
}

void
process_imu_record(struct psvr2_hmd *hmd, int index, struct imu_usb_record *in, timepoint_ns received_ns)
{
	struct imu_record imu_data;

	imu_data.vts_us = __le32_to_cpu(in->vts_us);
	for (int i = 0; i < 3; i++) {
		imu_data.accel[i] = __le16_to_cpu(in->accel[i]);
		imu_data.gyro[i] = __le16_to_cpu(in->gyro[i]);
	}
	imu_data.dp_frame_cnt = __le16_to_cpu(in->dp_frame_cnt);
	imu_data.dp_line_cnt = __le16_to_cpu(in->dp_line_cnt);
	imu_data.imu_ts_us = __le16_to_cpu(in->imu_ts_us);
	imu_data.status = __le16_to_cpu(in->status);

	PSVR2_TRACE(hmd,
	            "Record #%d: TS %u vts %u "
	            "accel { %d, %d, %d } gyro { %d, %d, %d } "
	            "dp_frame_cnt %u dp_line_cnt %u status %u",
	            index, imu_data.imu_ts_us, imu_data.vts_us, imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
	            imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2], imu_data.dp_frame_cnt, imu_data.dp_line_cnt,
	            imu_data.status);

	uint32_t last_vts_us = hmd->last_vts_us;
	uint32_t last_imu_ts = hmd->last_imu_ts;

	hmd->last_vts_us = imu_data.vts_us; /* Last VTS timestamp */
	hmd->last_imu_ts = imu_data.imu_ts_us;

	hmd->last_gyro.x = -DEG_TO_RAD(imu_data.gyro[1] * GYRO_SCALE);
	hmd->last_gyro.y = DEG_TO_RAD(imu_data.gyro[2] * GYRO_SCALE);
	hmd->last_gyro.z = -DEG_TO_RAD(imu_data.gyro[0] * GYRO_SCALE);

	hmd->last_accel.x = -imu_data.accel[1] * ACCEL_SCALE;
	hmd->last_accel.y = imu_data.accel[2] * ACCEL_SCALE;
	hmd->last_accel.z = -imu_data.accel[0] * ACCEL_SCALE;

	if (hmd->timestamp_initialized) {
		// @note Overflow expected and fine, since this is an unsigned subtraction
		uint32_t vts_delta_us = imu_data.vts_us - last_vts_us;
		uint16_t imu_delta_us = imu_data.imu_ts_us - last_imu_ts;

		hmd->last_vts_ns += (timepoint_ns)vts_delta_us * U_TIME_1US_IN_NS;
		hmd->last_imu_ns += (timepoint_ns)imu_delta_us * U_TIME_1US_IN_NS;

		const timepoint_ns now_hw = hmd->last_vts_ns;
		const timepoint_ns now_imu = hmd->last_imu_ns;
		const timepoint_ns now_mono = received_ns - hmd->system_zero_ns;

		const float IMU_FREQ = 2000.0f;
		m_clock_offset_a2b(IMU_FREQ, now_hw, now_mono, &hmd->hw2mono);
		m_clock_offset_a2b(IMU_FREQ, now_imu, now_mono, &hmd->hw2mono_imu);

		struct xrt_imu_sample sample = {
		    .timestamp_ns = hmd->last_vts_ns,
		    .accel_m_s2 = {hmd->last_accel.x, hmd->last_accel.y, hmd->last_accel.z},
		    .gyro_rad_secs = {hmd->last_gyro.x, hmd->last_gyro.y, hmd->last_gyro.z},
		};

		// TODO: process IMU samples into fusion
		(void)sample;
	}
}

static void
process_status_report(struct psvr2_hmd *hmd, uint8_t *buf, int bytes_read, timepoint_ns received_ns)
{
	struct status_record_hdr *hdr = (struct status_record_hdr *)buf;

	hmd->dprx_status = hdr->dprx_status;
	hmd->proximity_sensor = hdr->prox_sensor_flag;
	hmd->passthrough_button = hdr->passthrough_button;

	hmd->ipd_updated |= (hmd->ipd_mm != hdr->ipd_dial_mm);
	hmd->ipd_mm = hdr->ipd_dial_mm;

	int i = 0;
	uint8_t *cur = buf + sizeof(struct status_record_hdr);
	uint8_t *end = buf + bytes_read;
	while (cur < end) {
		if ((size_t)(end - cur) < sizeof(struct imu_usb_record)) {
			break;
		}

		struct imu_usb_record *imu = (struct imu_usb_record *)cur;
		process_imu_record(hmd, i, imu, received_ns);

		cur += sizeof(struct imu_usb_record);
		i++;
	}
}

static bool
hmd_usb_xfer_continue(struct libusb_transfer *xfer, const char *type)
{
	struct psvr2_hmd *hmd = xfer->user_data;

	switch (xfer->status) {
	case LIBUSB_TRANSFER_OVERFLOW:
		PSVR2_ERROR(hmd, "%s xfer returned overflow!", type);
		/* Fall through */
	case LIBUSB_TRANSFER_ERROR:
	case LIBUSB_TRANSFER_TIMED_OUT:
	case LIBUSB_TRANSFER_CANCELLED:
	case LIBUSB_TRANSFER_STALL:
	case LIBUSB_TRANSFER_NO_DEVICE:
		os_thread_helper_lock(&hmd->usb_thread);
		hmd->usb_active_xfers--;
		os_thread_helper_signal_locked(&hmd->usb_thread);
		os_thread_helper_unlock(&hmd->usb_thread);
		PSVR2_TRACE(hmd, "%s xfer is aborting with status %d", type, xfer->status);
		return false;

	case LIBUSB_TRANSFER_COMPLETED: break;
	}

	return true;
}

static void LIBUSB_CALL
status_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();

	if (!hmd_usb_xfer_continue(xfer, "Status")) {
		return;
	}

	timepoint_ns received_ns = os_monotonic_get_ns();

	/* handle status packet */
	struct psvr2_hmd *hmd = xfer->user_data;
	os_mutex_lock(&hmd->data_lock);
	if ((size_t)xfer->actual_length >= sizeof(struct status_record_hdr)) {
		PSVR2_TRACE(hmd, "Status - %d bytes", xfer->actual_length);
		PSVR2_TRACE_HEX(hmd, xfer->buffer, xfer->actual_length);

		process_status_report(hmd, xfer->buffer, xfer->actual_length, received_ns);
	}

	libusb_submit_transfer(xfer);
	os_mutex_unlock(&hmd->data_lock);
}

static void LIBUSB_CALL
img_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();

	if (!hmd_usb_xfer_continue(xfer, "Camera frame")) {
		return;
	}

	struct psvr2_hmd *hmd = xfer->user_data;
	if (xfer->actual_length > 0) {
		PSVR2_TRACE(hmd, "Camera frame - %d bytes", xfer->actual_length);
		PSVR2_TRACE_HEX(hmd, xfer->buffer, MIN(256, xfer->actual_length));

		if (xfer->actual_length == USB_CAM_MODE10_XFER_SIZE) {
			for (int d = 0; d < 3; d++) {
				if (u_sink_debug_is_active(&hmd->debug_sinks[d])) {
					struct xrt_frame *xf = NULL;

					int w = 254, h = 508, stride = 256, offset, size_pp;
					if (d == 0) {
						offset = d;
						size_pp = 2;
						u_frame_create_one_off(XRT_FORMAT_L8, stride * 2, h, &xf);
					} else if (d == 1 || d == 2) {
						offset = (d == 1) ? 2 : 5;
						size_pp = 3;
						u_frame_create_one_off(XRT_FORMAT_R8G8B8, stride, h, &xf);
					}

					uint8_t *src = xfer->buffer + 256;
					uint8_t *dest = xf->data;
					for (int y = 0; y < h; y++) {
						int x;

						for (x = 0; x < w; x++) {
							for (int i = 0; i < size_pp; i++) {
								*dest++ = src[offset + i];
							}
							src += 8;
						}
						src += 16; /* Skip 16-bytes at the end of each line */
						           /* Skip output padding pixels */
						while (x++ < stride) {
							for (int i = 0; i < size_pp; i++) {
								*dest++ = 0;
							}
						}
					}
					xf->timestamp = os_monotonic_get_ns();
					u_sink_debug_push_frame(&hmd->debug_sinks[d], xf);
					xrt_frame_reference(&xf, NULL);
				}
			}
		} else if (xfer->actual_length == USB_CAM_MODE1_XFER_SIZE) {
			if (u_sink_debug_is_active(&hmd->debug_sinks[3])) {

				struct xrt_frame *xf = NULL;
				u_frame_create_one_off(XRT_FORMAT_L8, 1280, 640, &xf);

				uint8_t *src = xfer->buffer + 256;
				uint8_t *dest = xf->data;
				memcpy(dest, src, 640 * 1280);
				xf->timestamp = os_monotonic_get_ns();
				u_sink_debug_push_frame(&hmd->debug_sinks[3], xf);
				xrt_frame_reference(&xf, NULL);
			}
		}
	}

	os_mutex_lock(&hmd->data_lock);
	libusb_submit_transfer(xfer);
	os_mutex_unlock(&hmd->data_lock);
}

static void
process_slam_record(struct psvr2_hmd *hmd, uint8_t *buf, int bytes_read)
{
	struct slam_usb_record *usb_data = (struct slam_usb_record *)buf;
	union {
		uint32_t i;
		float f;
	} u;

	assert(bytes_read >= (int)sizeof(struct slam_usb_record));

	struct slam_record slam;
	slam.ts_us = __le32_to_cpu(usb_data->ts);

	for (int i = 0; i < 3; i++) {
		u.i = __le32_to_cpu(usb_data->pos[i]);
		slam.pos[i] = u.f;
	}

	for (int i = 0; i < 4; i++) {
		u.i = __le32_to_cpu(usb_data->orient[i]);
		slam.orient[i] = u.f;
	}

	if (usb_data->unknown1 != 3) {
		PSVR2_DEBUG(hmd, "SLAM - unknown1 field was not 3, it was %d", usb_data->unknown1);
	}
	// assert(usb_data->unknown1 == 3 || usb_data->unknown1 == 0);

	os_mutex_lock(&hmd->data_lock);

	if (!hmd->timestamp_initialized) {
		// Initialize all timestamps on first SLAM frame
		hmd->system_zero_ns = os_monotonic_get_ns();
		hmd->last_vts_ns = 0;
		hmd->last_slam_ns = 0;
		hmd->last_imu_ns = 0;
		hmd->timestamp_initialized = true;
	} else {
		// @note Overflow expected and fine, since this is an unsigned subtraction
		uint32_t slam_ts_delta_us = slam.ts_us - hmd->last_slam_ts_us;

		hmd->last_slam_ns += (timepoint_ns)slam_ts_delta_us * U_TIME_1US_IN_NS;
	}

	//@todo: Manual axis correction should come from calibration somewhere I think
	hmd->last_slam_ts_us = slam.ts_us;
	hmd->last_slam_pose.position.x = slam.pos[2];
	hmd->last_slam_pose.position.y = slam.pos[1];
	hmd->last_slam_pose.position.z = -slam.pos[0];
	hmd->last_slam_pose.orientation.w = slam.orient[0];
	hmd->last_slam_pose.orientation.x = -slam.orient[2];
	hmd->last_slam_pose.orientation.y = -slam.orient[1];
	hmd->last_slam_pose.orientation.z = slam.orient[3];

	struct xrt_pose tmp = hmd->slam_correction_pose;
	math_quat_normalize(&tmp.orientation);
	math_quat_rotate(&tmp.orientation, &hmd->last_slam_pose.orientation, &hmd->pose.orientation);
	hmd->pose.position = hmd->last_slam_pose.position;
	math_vec3_accum(&tmp.position, &hmd->pose.position);
	os_mutex_unlock(&hmd->data_lock);

	PSVR2_DEBUG(hmd, "SLAM - %d leftover bytes", (int)sizeof(usb_data->remainder));
	PSVR2_DEBUG_HEX(hmd, usb_data->remainder, sizeof(usb_data->remainder));

	if (!hmd->timestamp_initialized) {
		int64_t now = os_monotonic_get_ns();
		hmd->timestamp_initialized = true;
		hmd->system_zero_ns = now;
	}

	struct xrt_pose_sample pose_sample = {
	    .timestamp_ns = hmd->last_slam_ns,
	    .pose = hmd->pose,
	};

	// TODO: process SLAM pose into fusion
	(void)pose_sample;
}

static void LIBUSB_CALL
slam_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();

	if (!hmd_usb_xfer_continue(xfer, "SLAM frame")) {
		return;
	}

	struct psvr2_hmd *hmd = xfer->user_data;
	if (xfer->actual_length == sizeof(struct slam_usb_record)) {
		process_slam_record(hmd, xfer->buffer, xfer->actual_length);
	}

	os_mutex_lock(&hmd->data_lock);
	libusb_submit_transfer(xfer);
	os_mutex_unlock(&hmd->data_lock);
}

static void LIBUSB_CALL
dump_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();
	struct psvr2_hmd *hmd = xfer->user_data;
	const char *name = NULL;

	if (xfer == hmd->led_detector_xfer)
		name = "LED Detector";
	else if (xfer == hmd->relocalizer_xfer)
		name = "RP";
	else if (xfer == hmd->vd_xfer)
		name = "VD";
	assert(name != NULL);

	if (!hmd_usb_xfer_continue(xfer, name)) {
		return;
	}

	PSVR2_DEBUG(hmd, "%s xfer size %u", name, xfer->actual_length);
	PSVR2_TRACE_HEX(hmd, xfer->buffer, xfer->actual_length);

	os_mutex_lock(&hmd->data_lock);
	libusb_submit_transfer(xfer);
	os_mutex_unlock(&hmd->data_lock);
}

static void *
psvr2_usb_thread(void *ptr)
{
	U_TRACE_SET_THREAD_NAME("PSVR2: USB communication");

	struct psvr2_hmd *hmd = ptr;

	os_thread_helper_lock(&hmd->usb_thread);
	while (os_thread_helper_is_running_locked(&hmd->usb_thread) && !hmd->usb_complete) {
		os_thread_helper_unlock(&hmd->usb_thread);

		libusb_handle_events_completed(hmd->ctx, &hmd->usb_complete);

		os_thread_helper_lock(&hmd->usb_thread);
	}

	os_thread_helper_unlock(&hmd->usb_thread);

	// Shut down USB communication
	psvr2_usb_stop(hmd);

	libusb_handle_events(hmd->ctx);

	return NULL;
}

struct
{
	int interface_no;
	int altmode;
	const char *name;
} interface_list[] = {
    {.interface_no = PSVR2_STATUS_INTERFACE, .altmode = 1, .name = "status"},
    {.interface_no = PSVR2_SLAM_INTERFACE, .altmode = 0, .name = "SLAM"},
    {.interface_no = PSVR2_CAMERA_INTERFACE, .altmode = 0, .name = "Camera"},
    {.interface_no = PSVR2_LD_INTERFACE, .altmode = 0, .name = "LED Detector"},
    {.interface_no = PSVR2_RP_INTERFACE, .altmode = 0, .name = "Relocalizer"},
    {.interface_no = PSVR2_VD_INTERFACE, .altmode = 0, .name = "VD"},
};

static bool
psvr2_usb_open(struct psvr2_hmd *hmd, struct xrt_prober_device *xpdev)
{
	int res;

	res = libusb_init(&hmd->ctx);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Failed to init USB");
		return false;
	}

	hmd->dev = libusb_open_device_with_vid_pid(hmd->ctx, xpdev->vendor_id, xpdev->product_id);
	if (hmd->dev == NULL) {
		PSVR2_ERROR(hmd, "Failed to open USB device");
		return false;
	}

	for (size_t i = 0; i < sizeof(interface_list) / sizeof(interface_list[0]); i++) {
		int intf_no = interface_list[i].interface_no;
		int altmode = interface_list[i].altmode;
		const char *name = interface_list[i].name;

		res = libusb_claim_interface(hmd->dev, intf_no);
		if (res < 0) {
			PSVR2_ERROR(hmd, "Failed to claim USB %s interface", name);
			return false;
		}
		res = libusb_set_interface_alt_setting(hmd->dev, intf_no, altmode);
		if (res < 0) {
			PSVR2_ERROR(hmd, "Failed to set USB %s interface alt %d", name, altmode);
			return false;
		}
	}

	return true;
}

bool
get_psvr2_control(struct psvr2_hmd *hmd, uint16_t report_id, uint8_t subcmd, uint8_t *out_data, uint32_t buf_size)
{
	struct sie_ctrl_pkt pkt = {0};
	int ret;

	assert(buf_size <= sizeof(pkt.data));

	pkt.report_id = __cpu_to_le16(report_id);
	pkt.subcmd = __cpu_to_le16(subcmd);
	pkt.len = __cpu_to_le32(buf_size);

	ret = libusb_control_transfer(hmd->dev, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT | 0x80, 0x1,
	                              report_id, 0x0, (unsigned char *)&pkt, buf_size + 8, 100);
	if (ret < 0) {
		PSVR2_ERROR(hmd, "Failed to get report id %u subcmd %u, reason %d", report_id, subcmd, ret);
		return false;
	}

	memcpy(out_data, pkt.data, buf_size);

	return true;
}

bool
send_psvr2_control(struct psvr2_hmd *hmd, uint16_t report_id, uint8_t subcmd, uint8_t *pkt_data, uint32_t pkt_len)
{
	struct sie_ctrl_pkt pkt;
	int ret;

	assert(pkt_len <= sizeof(pkt.data));

	pkt.report_id = __cpu_to_le16(report_id);
	pkt.subcmd = __cpu_to_le16(subcmd);
	pkt.len = __cpu_to_le32(pkt_len);
	memcpy(pkt.data, pkt_data, pkt_len);

	ret = libusb_control_transfer(hmd->dev, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT, 0x9, report_id,
	                              0x0, (unsigned char *)&pkt, pkt_len + 8, 100);
	if (ret < 0) {
		PSVR2_ERROR(hmd, "Failed to send report id %u subcmd %u", report_id, subcmd);
		return false;
	}

	return true;
}

bool
set_camera_mode(struct psvr2_hmd *hmd, enum psvr2_camera_mode mode)
{
	struct camera_cmd
	{
		__le32 data[2];
	} cmd;

	cmd.data[0] = __cpu_to_le32(0x1);
	cmd.data[1] = __cpu_to_le32(mode);

	return send_psvr2_control(hmd, 0xB, 0x1, (uint8_t *)(&cmd), sizeof(cmd));
}

static void
toggle_camera_enable(struct psvr2_hmd *hmd)
{
	hmd->camera_enable = !hmd->camera_enable;

	struct u_var_button *btn = &hmd->camera_enable_btn;
	snprintf(btn->label, sizeof(btn->label),
	         hmd->camera_enable ? "Disable camera streams" : "Enable camera streams");

	if (hmd->camera_enable) {
		set_camera_mode(hmd, hmd->camera_mode);
	} else {
		set_camera_mode(hmd, PSVR2_CAMERA_MODE_OFF);
	}
}

static void
cycle_camera_mode(struct psvr2_hmd *hmd)
{
	struct u_var_button *btn = &hmd->camera_mode_btn;

	switch (hmd->camera_mode) {
	case PSVR2_CAMERA_MODE_OFF:
	case PSVR2_CAMERA_MODE_1:
		hmd->camera_mode++;
		snprintf(btn->label, sizeof(btn->label), "Camera Mode 0x%x", hmd->camera_mode);
		break;
	case PSVR2_CAMERA_MODE_10:
		hmd->camera_mode = PSVR2_CAMERA_MODE_1;
		snprintf(btn->label, sizeof(btn->label), "Camera Mode 0x1");
		break;
	}

	if (hmd->camera_enable) {
		set_camera_mode(hmd, hmd->camera_mode);
	} else {
		set_camera_mode(hmd, PSVR2_CAMERA_MODE_OFF);
	}
}


static bool
psvr2_usb_start(struct psvr2_hmd *hmd)
{
	bool result = false;
	int res;

	os_thread_helper_lock(&hmd->usb_thread);

	/* Status endpoint */
	hmd->status_xfer = libusb_alloc_transfer(0);
	if (hmd->status_xfer == NULL) {
		PSVR2_ERROR(hmd, "Could not alloc USB transfer for status reports");
		goto out;
	}
	uint8_t *status_buf = malloc(USB_STATUS_XFER_SIZE);
	libusb_fill_interrupt_transfer(hmd->status_xfer, hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_STATUS_ENDPOINT,
	                               status_buf, USB_STATUS_XFER_SIZE, status_xfer_cb, hmd, 0);
	hmd->status_xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

	res = libusb_submit_transfer(hmd->status_xfer);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not submit USB transfer for status reports");
		goto out;
	}
	hmd->usb_active_xfers++;

	/* Camera data */
	hmd->camera_enable = true;
	hmd->camera_mode = PSVR2_CAMERA_MODE_10;
	set_camera_mode(hmd, hmd->camera_mode);

	for (int i = 0; i < NUM_CAM_XFERS; i++) {
		hmd->camera_xfers[i] = libusb_alloc_transfer(0);
		if (hmd->camera_xfers[i] == NULL) {
			PSVR2_ERROR(hmd, "Could not alloc USB transfer %d for camera data", i);
			goto out;
		}

		uint8_t *recv_buf = malloc(USB_CAM_MODE10_XFER_SIZE);

		libusb_fill_bulk_transfer(hmd->camera_xfers[i], hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_CAMERA_ENDPOINT,
		                          recv_buf, USB_CAM_MODE10_XFER_SIZE, img_xfer_cb, hmd, 0);
		hmd->camera_xfers[i]->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

		res = libusb_submit_transfer(hmd->camera_xfers[i]);
		if (res < 0) {
			PSVR2_ERROR(hmd, "Could not submit USB transfer %d for camera data", i);
			goto out;
		}
		hmd->usb_active_xfers++;
	}

	/* SLAM endpoint */
	hmd->slam_xfer = libusb_alloc_transfer(0);
	if (hmd->slam_xfer == NULL) {
		PSVR2_ERROR(hmd, "Could not alloc USB transfer for SLAM data");
		goto out;
	}
	uint8_t *slam_buf = malloc(USB_SLAM_XFER_SIZE);
	libusb_fill_bulk_transfer(hmd->slam_xfer, hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_SLAM_ENDPOINT, slam_buf,
	                          USB_SLAM_XFER_SIZE, slam_xfer_cb, hmd, 0);
	hmd->slam_xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

	res = libusb_submit_transfer(hmd->slam_xfer);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not submit USB transfer for SLAM data");
		goto out;
	}
	hmd->usb_active_xfers++;

	/* LD endpoint */
	hmd->led_detector_xfer = libusb_alloc_transfer(0);
	if (hmd->led_detector_xfer == NULL) {
		PSVR2_ERROR(hmd, "Could not alloc USB transfer for LED Detector data");
		goto out;
	}
	uint8_t *led_detector_buf = malloc(USB_LD_XFER_SIZE);
	libusb_fill_bulk_transfer(hmd->led_detector_xfer, hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_LD_ENDPOINT,
	                          led_detector_buf, USB_LD_XFER_SIZE, dump_xfer_cb, hmd, 0);
	hmd->led_detector_xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

	res = libusb_submit_transfer(hmd->led_detector_xfer);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not submit USB transfer for LED Detector data");
		goto out;
	}
	hmd->usb_active_xfers++;

	/* RP endpoint */
	hmd->relocalizer_xfer = libusb_alloc_transfer(0);
	if (hmd->relocalizer_xfer == NULL) {
		PSVR2_ERROR(hmd, "Could not alloc USB transfer for RP data");
		goto out;
	}
	uint8_t *relocalizer_buf = malloc(USB_RP_XFER_SIZE);
	libusb_fill_bulk_transfer(hmd->relocalizer_xfer, hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_RP_ENDPOINT,
	                          relocalizer_buf, USB_RP_XFER_SIZE, dump_xfer_cb, hmd, 0);
	hmd->relocalizer_xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

	res = libusb_submit_transfer(hmd->relocalizer_xfer);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not submit USB transfer for RP data");
		goto out;
	}
	hmd->usb_active_xfers++;

	/* VD endpoint */
	hmd->vd_xfer = libusb_alloc_transfer(0);
	if (hmd->vd_xfer == NULL) {
		PSVR2_ERROR(hmd, "Could not alloc USB transfer for VD data");
		goto out;
	}
	uint8_t *vd_buf = malloc(USB_VD_XFER_SIZE);
	libusb_fill_bulk_transfer(hmd->vd_xfer, hmd->dev, LIBUSB_ENDPOINT_IN | PSVR2_VD_ENDPOINT, vd_buf,
	                          USB_VD_XFER_SIZE, dump_xfer_cb, hmd, 0);
	hmd->vd_xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

	res = libusb_submit_transfer(hmd->vd_xfer);
	if (res < 0) {
		PSVR2_ERROR(hmd, "Could not submit USB transfer for VD data");
		goto out;
	}
	hmd->usb_active_xfers++;


	result = true;

out:
	os_thread_helper_unlock(&hmd->usb_thread);
	return result;
}

static void
set_slam_correction(struct psvr2_hmd *hmd)
{
	os_mutex_lock(&hmd->data_lock);
	math_pose_invert(&hmd->last_slam_pose, &hmd->slam_correction_pose);
	os_mutex_unlock(&hmd->data_lock);
}

static void
reset_slam_correction(struct psvr2_hmd *hmd)
{
	os_mutex_lock(&hmd->data_lock);
	hmd->slam_correction_pose = (struct xrt_pose)SLAM_POSE_CORRECTION;
	os_mutex_unlock(&hmd->data_lock);
}

static void
psvr2_usb_stop(struct psvr2_hmd *hmd)
{
	int ret;

	os_mutex_lock(&hmd->data_lock);
	if (hmd->vd_xfer) {
		ret = libusb_cancel_transfer(hmd->vd_xfer);
		assert(ret == 0 || ret == LIBUSB_ERROR_NOT_FOUND);
	}
	if (hmd->relocalizer_xfer) {
		ret = libusb_cancel_transfer(hmd->relocalizer_xfer);
		assert(ret == 0 || ret == LIBUSB_ERROR_NOT_FOUND);
	}
	if (hmd->led_detector_xfer) {
		ret = libusb_cancel_transfer(hmd->led_detector_xfer);
		assert(ret == 0 || ret == LIBUSB_ERROR_NOT_FOUND);
	}
	for (int i = 0; i < NUM_CAM_XFERS; i++) {
		if (hmd->camera_xfers[i]) {
			ret = libusb_cancel_transfer(hmd->camera_xfers[i]);
			assert(ret == 0 || ret == LIBUSB_ERROR_NOT_FOUND);
		}
	}
	if (hmd->slam_xfer) {
		ret = libusb_cancel_transfer(hmd->slam_xfer);
		assert(ret == 0 || ret == LIBUSB_ERROR_NOT_FOUND);
	}
	if (hmd->status_xfer) {
		ret = libusb_cancel_transfer(hmd->status_xfer);
		assert(ret == 0 || ret == LIBUSB_ERROR_NOT_FOUND);
	}

	os_mutex_unlock(&hmd->data_lock);
}

void
psvr2_usb_destroy(struct psvr2_hmd *hmd)
{
	// All transfers are stopped and can be freed now
	if (hmd->status_xfer) {
		libusb_free_transfer(hmd->status_xfer);
		hmd->status_xfer = NULL;
	}
	for (int i = 0; i < NUM_CAM_XFERS; i++) {
		if (hmd->camera_xfers[i]) {
			libusb_free_transfer(hmd->camera_xfers[i]);
			hmd->camera_xfers[i] = NULL;
		}
	}
	if (hmd->slam_xfer) {
		libusb_free_transfer(hmd->slam_xfer);
		hmd->slam_xfer = NULL;
	}
	if (hmd->led_detector_xfer) {
		libusb_free_transfer(hmd->led_detector_xfer);
		hmd->slam_xfer = NULL;
	}
	if (hmd->relocalizer_xfer) {
		libusb_free_transfer(hmd->relocalizer_xfer);
		hmd->slam_xfer = NULL;
	}
	if (hmd->vd_xfer) {
		libusb_free_transfer(hmd->vd_xfer);
		hmd->slam_xfer = NULL;
	}
}

struct distortion_calibration_block
{
	uint8_t version_unk;
	uint8_t unk[7];
	float distortion_params[32];
};

static void
psvr2_setup_distortion_and_fovs(struct psvr2_hmd *hmd)
{
	/* Each eye has an X offset, a Y offset, and two scale
	 * factors (the main scale factor, and another that
	 * allows for tilting the view, set to 0 for no tilt).
	 * It seems to be stored like this:
	 * struct calibration_t {
	 *	float offsetx_left;
	 *	float offsety_left;
	 *	float offsetx_right;
	 *	float offsety_right;
	 *	float scale1_left;
	 *	float scale2_left;
	 *	float scale1_right;
	 *	float scale2_right;
	 * } calibration;
	 * */
	struct distortion_calibration_block calibration_block;

	uint8_t buf[0x100];
	get_psvr2_control(hmd, 0x8f, 1, buf, sizeof buf);
	memcpy(&calibration_block, buf, sizeof calibration_block);

	memset(hmd->distortion_calibration, 0, sizeof(hmd->distortion_calibration));
	if (calibration_block.version_unk < 4) {
		hmd->distortion_calibration[0] = -0.09919293;
		hmd->distortion_calibration[2] = 0.09919293;
	} else {
		hmd->distortion_calibration[0] =
		    (((-calibration_block.distortion_params[0] - calibration_block.distortion_params[6]) * 29.9 +
		      14.95) /
		         1000.0 -
		     3.22) /
		    32.46199;
		hmd->distortion_calibration[1] =
		    (((-calibration_block.distortion_params[1] * 29.9) + 14.95) / 1000.0) / 32.46199;

		hmd->distortion_calibration[2] =
		    (((calibration_block.distortion_params[6] - calibration_block.distortion_params[2]) * 29.9 +
		      14.95) /
		         1000.0 +
		     3.22) /
		    32.46199;
		hmd->distortion_calibration[3] =
		    (((-calibration_block.distortion_params[3] * 29.9) + 14.95) / 1000.0) / 32.46199;

		float left = -calibration_block.distortion_params[4] * M_PI / 180.0;
		hmd->distortion_calibration[4] = cosf(left);
		hmd->distortion_calibration[5] = sinf(left);

		float right = -calibration_block.distortion_params[5] * M_PI / 180.0;
		hmd->distortion_calibration[6] = cosf(right);
		hmd->distortion_calibration[7] = sinf(right);
	}

	struct xrt_fov *fovs = hmd->base.hmd->distortion.fov;
	fovs[0].angle_up = 53.0f * (M_PI / 180.0f);
	fovs[0].angle_down = -53.0f * (M_PI / 180.0f);
	fovs[0].angle_left = -61.5f * (M_PI / 180.0f);
	fovs[0].angle_right = 43.5f * (M_PI / 180.0f);

	fovs[1].angle_up = fovs[0].angle_up;
	fovs[1].angle_down = fovs[0].angle_down;
	fovs[1].angle_left = -fovs[0].angle_right;
	fovs[1].angle_right = -fovs[0].angle_left;
}

struct xrt_device *
psvr2_hmd_create(struct xrt_prober_device *xpdev)
{
	DRV_TRACE_MARKER();

	// This indicates you won't be using Monado's built-in tracking algorithms.
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct psvr2_hmd *hmd = U_DEVICE_ALLOCATE(struct psvr2_hmd, flags, 1, 0);

	if (os_mutex_init(&hmd->data_lock) != 0) {
		PSVR2_ERROR(hmd, "Failed to init data mutex!");
		goto cleanup;
	}

	if (os_thread_helper_init(&hmd->usb_thread) != 0) {
		PSVR2_ERROR(hmd, "Failed to initialise threading");
		goto cleanup;
	}

	if (!psvr2_usb_open(hmd, xpdev)) {
		goto cleanup;
	}

	// This list should be ordered, most preferred first.
	size_t idx = 0;
	hmd->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = idx;

	hmd->base.update_inputs = psvr2_hmd_update_inputs;
	hmd->base.get_tracked_pose = psvr2_hmd_get_tracked_pose;
	hmd->base.get_view_poses = psvr2_hmd_get_view_poses;
	hmd->base.destroy = psvr2_hmd_destroy;

	hmd->pose = (struct xrt_pose)XRT_POSE_IDENTITY;
	hmd->log_level = debug_get_log_option_psvr2_log();

	// Print name.
	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "PS VR2 HMD");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "PS VR2 HMD S/N"); //! @todo Add serial number

	// Setup input.
	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.orientation_tracking_supported = true;
	hmd->base.position_tracking_supported = false;

	// Set up display details
	// refresh rate
	hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / 120.0f);
	hmd->base.compute_distortion = psvr2_compute_distortion;

	struct xrt_hmd_parts *parts = hmd->base.hmd;
	parts->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	parts->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;

	// This default matches the default lens separation
	hmd->ipd_mm = 65;

	hmd->info.display.w_pixels = 4000;
	hmd->info.display.h_pixels = 2040;
	hmd->info.display.w_meters = 0.13f;
	hmd->info.display.h_meters = 0.07f;
	hmd->info.lens_horizontal_separation_meters = 0.13f / 2.0f;
	hmd->info.lens_vertical_position_meters = 0.07f / 2.0f;
	// These need to be set to avoid an error, but the fovs
	// computed further down are preferred.
	hmd->info.fov[0] = (float)(106.0 * (M_PI / 180.0));
	hmd->info.fov[1] = (float)(106.0 * (M_PI / 180.0));

	if (!u_device_setup_split_side_by_side(&hmd->base, &hmd->info)) {
		PSVR2_ERROR(hmd, "Failed to setup basic device info");
		goto cleanup;
	}

	psvr2_setup_distortion_and_fovs(hmd);

	u_distortion_mesh_fill_in_compute(&hmd->base);

	const struct xrt_pose slam_correction_pose = SLAM_POSE_CORRECTION;
	hmd->slam_correction_pose = slam_correction_pose;

	for (int i = 0; i < 4; i++) {
		u_sink_debug_init(&hmd->debug_sinks[i]);
	}

	u_var_add_root(hmd, "PS VR2 HMD", true);
	u_var_add_pose(hmd, &hmd->pose, "pose");
	u_var_add_pose(hmd, &hmd->slam_correction_pose, "SLAM correction pose");
	{
		hmd->slam_correction_set_btn.cb = (void (*)(void *))set_slam_correction;
		hmd->slam_correction_set_btn.ptr = hmd;
		u_var_add_button(hmd, &hmd->slam_correction_set_btn, "Set");
	}
	{
		hmd->slam_correction_reset_btn.cb = (void (*)(void *))reset_slam_correction;
		hmd->slam_correction_reset_btn.ptr = hmd;
		u_var_add_button(hmd, &hmd->slam_correction_reset_btn, "Reset");
	}

	u_var_add_gui_header(hmd, NULL, "Last IMU data");
	u_var_add_ro_u32(hmd, &hmd->last_vts_us, "VTS Timestamp");
	u_var_add_u16(hmd, &hmd->last_imu_ts, "Timestamp");
	u_var_add_ro_vec3_f32(hmd, &hmd->last_accel, "accel");
	u_var_add_ro_vec3_f32(hmd, &hmd->last_gyro, "gyro");

	u_var_add_gui_header(hmd, NULL, "Last SLAM data");
	u_var_add_ro_u32(hmd, &hmd->last_slam_ts_us, "Timestamp");
	u_var_add_pose(hmd, &hmd->last_slam_pose, "Pose");

	u_var_add_gui_header(hmd, NULL, "Status");
	u_var_add_u8(hmd, &hmd->dprx_status, "HMD Display Port RX status");
	u_var_add_bool(hmd, &hmd->proximity_sensor, "HMD Proximity");
	u_var_add_bool(hmd, &hmd->passthrough_button, "HMD Passthrough button");
	u_var_add_u8(hmd, &hmd->ipd_mm, "HMD IPD (mm)");

	u_var_add_gui_header(hmd, NULL, "Camera data");
	{
		hmd->camera_enable_btn.cb = (void (*)(void *))toggle_camera_enable;
		hmd->camera_enable_btn.ptr = hmd;
		u_var_add_button(hmd, &hmd->camera_enable_btn, "Disable camera streams");

		hmd->camera_mode_btn.cb = (void (*)(void *))cycle_camera_mode;
		hmd->camera_mode_btn.ptr = hmd;
		u_var_add_button(hmd, &hmd->camera_mode_btn, "Camera Mode 0x10");
	}
	for (int i = 0; i < 3; i++) {
		char name[32];
		sprintf(name, "Substream %d", i);
		u_var_add_sink_debug(hmd, &hmd->debug_sinks[i], name);
	}
	u_var_add_sink_debug(hmd, &hmd->debug_sinks[3], "Mode 1 stream");

	u_var_add_gui_header(hmd, NULL, "Logging");
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");

	// Start USB communications
	hmd->usb_complete = 0;
	if (os_thread_helper_start(&hmd->usb_thread, psvr2_usb_thread, hmd) != 0) {
		PSVR2_ERROR(hmd, "Failed to start USB thread");
		goto cleanup;
	}

	if (!psvr2_usb_start(hmd)) {
		PSVR2_ERROR(hmd, "Failed to submit USB transfers");
		goto cleanup;
	}

	return &hmd->base;

cleanup:
	psvr2_hmd_destroy(&hmd->base);
	return NULL;
}
