// Copyright 2025, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to Oculus Rift driver code.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#pragma once

#include "xrt/xrt_device.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_prober.h"

#include "os/os_hid.h"
#include "os/os_threading.h"

#include "util/u_device.h"
#include "util/u_logging.h"

#include "math/m_imu_3dof.h"
#include "math/m_api.h"
#include "math/m_mathinclude.h"

#include <stdlib.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define REPORT_MAX_SIZE 69                // max size of a feature report (FEATURE_REPORT_CALIBRATE)
#define KEEPALIVE_INTERVAL_NS 10000000000 // 10 seconds
// give a 5% breathing room (at 10 seconds, this is 500 milliseconds of breathing room)
#define KEEPALIVE_SEND_RATE_NS ((KEEPALIVE_INTERVAL_NS * 19) / 20)
#define IMU_SAMPLE_RATE (1000)      // 1000hz
#define NS_PER_SAMPLE (1000 * 1000) // 1ms (1,000,000 ns) per sample

#define DEG_TO_RAD(DEG) (DEG * M_PI / 180.0)
#define MICROMETERS_TO_METERS(microns) (float)microns / 1000000.0f

// value taken from LibOVR 0.4.4
#define DEFAULT_EXTRA_EYE_ROTATION DEG_TO_RAD(30.0f)

enum rift_feature_reports
{
	// DK1
	FEATURE_REPORT_CONFIG = 2,         // get + set
	FEATURE_REPORT_CALIBRATE = 3,      // get + set
	FEATURE_REPORT_RANGE = 4,          // get + set
	FEATURE_REPORT_REGISTER = 5,       // get + set
	FEATURE_REPORT_DFU = 6,            // get + set
	FEATURE_REPORT_DK1_KEEP_ALIVE = 8, // get + set
	FEATURE_REPORT_DISPLAY_INFO = 9,   // get + set
	FEATURE_REPORT_SERIAL = 10,        // get + set

	// DK2
	FEATURE_REPORT_TRACKING = 12,        // get + set
	FEATURE_REPORT_DISPLAY = 13,         // get + set
	FEATURE_REPORT_MAG_CALIBRATION = 14, // get + set
	FEATURE_REPORT_POS_CALIBRATION = 15, // get + set
	FEATURE_REPORT_CUSTOM_PATTERN = 16,  // get + set
	FEATURE_REPORT_KEEPALIVE_MUX = 17,   // get + set
	FEATURE_REPORT_MANUFACTURING = 18,   // get + set
	FEATURE_REPORT_UUID = 19,            // get + set
	FEATURE_REPORT_TEMPERATURE = 20,     // get + set
	FEATURE_REPORT_GYROOFFSET = 21,      // get only
	FEATURE_REPORT_LENS_DISTORTION = 22, // get + set
};

enum rift_config_report_flags
{
	// output the sample data raw from the sensors without converting them to known units
	RIFT_CONFIG_REPORT_USE_RAW = 1,
	// internal test mode for calibrating zero rate drift on gyro
	RIFT_CONFIG_REPORT_INTERNAL_CALIBRATION = 1 << 1,
	// use the calibration parameters stored on the device
	RIFT_CONFIG_REPORT_USE_CALIBRATION = 1 << 2,
	// recalibrate the gyro zero rate offset when the device is stationary
	RIFT_CONFIG_REPORT_AUTO_CALIBRATION = 1 << 3,
	// stop sending IN reports when the device has stopped moving for Interval milliseconds
	RIFT_CONFIG_REPORT_MOTION_KEEP_ALIVE = 1 << 4,
	// stop sending IN reports when the device has stopped receiving feature reports for Interval milliseconds
	RIFT_CONFIG_REPORT_COMMAND_KEEP_ALIVE = 1 << 5,
	// output the IN report data in the coordinate system used by LibOVR relative to the tracker, otherwise, report
	// in the coordinate system of the device
	RIFT_CONFIG_REPORT_USE_SENSOR_COORDINATES = 1 << 6,
	// override the power state of the USB hub, forcing it to act as if the external power source is connected (DK2
	// only, does nothing on DK1)
	RIFT_CONFIG_REPORT_OVERRIDE_POWER = 1 << 7,
};

enum rift_distortion_type
{
	RIFT_DISTORTION_TYPE_DIMS = 1,
	RIFT_DISTORTION_TYPE_K = 2,
};

enum rift_lens_type
{
	// firmware indirectly states lens type A is 0
	RIFT_LENS_TYPE_A = 0,
	// firmware does not state what lens type B is, 1 is an educated guess
	RIFT_LENS_TYPE_B = 1,
};

#define IN_REPORT_DK2 11

#define CATMULL_COEFFICIENTS 11
#define CHROMATIC_ABBERATION_COEFFEICENT_COUNT 4

enum rift_lens_distortion_version
{
	// no distortion data is stored
	RIFT_LENS_DISTORTION_NONE = 0,
	// standard distortion matrix
	RIFT_LENS_DISTORTION_LCSV_CATMULL_ROM_10_VERSION_1 = 1,
};

/*
 *
 * Packed structs for USB communication (borrowed from Rokid driver)
 *
 */

#if defined(__GNUC__)
#define RIFT_PACKED __attribute__((packed))
#else
#define RIFT_PACKED
#endif /* __GNUC__ */

#if defined(_MSC_VER)
#pragma pack(push, 1)
#endif

struct rift_config_report
{
	uint16_t command_id;
	uint8_t config_flags;
	// the IN report rate of the headset, rate is calculated as `sample_rate / (1 + interval)`
	uint8_t interval;
	// sample rate of the IMU, always 1000hz on DK1/DK2, read-only
	uint16_t sample_rate;
} RIFT_PACKED;

struct rift_display_info_report
{
	uint16_t command_id;
	uint8_t distortion_type;
	// the horizontal resolution of the display, in pixels
	uint16_t resolution_x;
	// the vertical resolution of the display, in pixels
	uint16_t resolution_y;
	// width in micrometers
	uint32_t display_width;
	// height in micrometers
	uint32_t display_height;
	// the vertical center of the display, in micrometers
	uint32_t center_v;
	// the separation between the two lenses, in micrometers
	uint32_t lens_separation;
	uint32_t lens_distance[2];
	float distortion[6];
} RIFT_PACKED;

struct rift_catmull_rom_distortion_report_data
{
	// eye relief setting, in micrometers from front surface of lens
	uint16_t eye_relief;
	// the k coeffecients of the distortion
	uint16_t k[CATMULL_COEFFICIENTS];
	uint16_t max_r;
	uint16_t meters_per_tan_angle_at_center;
	uint16_t chromatic_abberation[CHROMATIC_ABBERATION_COEFFEICENT_COUNT];
	uint8_t unused[14];
} RIFT_PACKED;

struct rift_lens_distortion_report
{
	uint16_t command_id;
	// the amount of distortions on this device
	uint8_t num_distortions;
	// the index of this distortion in the devices array
	uint8_t distortion_idx;
	// unused bitmask field
	uint8_t bitmask;
	// the type of the lenses
	uint16_t lens_type;
	// the version of the lens distortion data
	uint16_t distortion_version;

	union {
		struct rift_catmull_rom_distortion_report_data lcsv_catmull_rom_10;
	} RIFT_PACKED data;
} RIFT_PACKED;

struct dk2_report_keepalive_mux
{
	uint16_t command;
	uint8_t in_report;
	uint16_t interval;
} RIFT_PACKED;

enum rift_display_mode
{
	RIFT_DISPLAY_MODE_GLOBAL,
	RIFT_DISPLAY_MODE_ROLLING_TOP_BOTTOM,
	RIFT_DISPLAY_MODE_ROLLING_LEFT_RIGHT,
	RIFT_DISPLAY_MODE_ROLLING_RIGHT_LEFT,
};

enum rift_display_limit
{
	RIFT_DISPLAY_LIMIT_ACL_OFF = 0,
	RIFT_DISPLAY_LIMIT_ACL_30 = 1,
	RIFT_DISPLAY_LIMIT_ACL_25 = 2,
	RIFT_DISPLAY_LIMIT_ACL_50 = 3,
};

enum rift_display_flags
{
	RIFT_DISPLAY_USE_ROLLING = 1 << 6,
	RIFT_DISPLAY_REVERSE_ROLLING = 1 << 7,
	RIFT_DISPLAY_HIGH_BRIGHTNESS = 1 << 8,
	RIFT_DISPLAY_SELF_REFRESH = 1 << 9,
	RIFT_DISPLAY_READ_PIXEL = 1 << 10,
	RIFT_DISPLAY_DIRECT_PENTILE = 1 << 11,
};

struct rift_display_report
{
	uint16_t command_id;
	// relative brightness setting independent of pixel persistence, only effective when high brightness is disabled
	uint8_t brightness;
	// a set of flags, ordered from LSB -> MSB
	// - panel mode/shutter type (4 bits), read only, see rift_display_mode
	// - current limit (2 bits), see rift_display_limit
	// - use rolling (1 bit)
	// - reverse rolling (1 bit), unavailable on released DK2 firmware for unknown reason
	// - high brightness (1 bit), unavailable on released DK2 firmware for unpublished reason
	// - self refresh (1 bit)
	// - read pixel (1 bit)
	// - direct pentile (1 bit)
	uint32_t flags;
	// the length of time in rows that the display is lit each frame, defaults to the full size of the display, full
	// persistence
	uint16_t persistence;
	// the offset in rows from vsync that the panel is lit when using global shutter, no effect in rolling shutter,
	// disabled on released DK2 firmware for unknown reason
	uint16_t lighting_offset;
	// the time in microseconds it is estimated for a pixel to settle to one value after it is set, read only
	uint16_t pixel_settle;
	// the number of rows including active area and blanking period used with persistence and lightingoffset, read
	// only
	uint16_t total_rows;
} RIFT_PACKED;

struct dk2_sensor_sample
{
	uint8_t data[8];
} RIFT_PACKED;

struct dk2_sample_pack
{
	struct dk2_sensor_sample accel;
	struct dk2_sensor_sample gyro;
} RIFT_PACKED;

#define DK2_MAX_SAMPLES 2
struct dk2_in_report
{
	uint16_t command_id;
	uint8_t num_samples;
	uint16_t sample_count;
	uint16_t temperature;
	uint32_t sample_timestamp;
	struct dk2_sample_pack samples[DK2_MAX_SAMPLES];
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
	uint16_t frame_count;
	uint32_t frame_timestamp;
	uint8_t frame_id;
	uint8_t tracking_pattern;
	uint16_t tracking_count;
	uint32_t tracking_timestamp;
} RIFT_PACKED;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif

struct rift_catmull_rom_distortion_data
{
	// the k coeffecients of the distortion
	float k[CATMULL_COEFFICIENTS];
	float max_r;
	float meters_per_tan_angle_at_center;
	float chromatic_abberation[CHROMATIC_ABBERATION_COEFFEICENT_COUNT];
};

struct rift_lens_distortion
{
	// the version of the lens distortion data
	uint16_t distortion_version;
	// eye relief setting, in meters from surface of lens
	float eye_relief;

	union {
		struct rift_catmull_rom_distortion_data lcsv_catmull_rom_10;
	} RIFT_PACKED data;
};

struct rift_scale_and_offset
{
	struct xrt_vec2 scale;
	struct xrt_vec2 offset;
};

struct rift_viewport_fov_tan
{
	float up_tan;
	float down_tan;
	float left_tan;
	float right_tan;
};

struct rift_extra_display_info
{
	// gap left between the two eyes
	float screen_gap_meters;
	// the diameter of the lenses, may need to be extended to an array
	float lens_diameter_meters;
	// ipd of the headset
	float icd;

	// the fov of the headset
	struct rift_viewport_fov_tan fov;
	// mapping from tan-angle space to target NDC space
	struct rift_scale_and_offset eye_to_source_ndc;
	struct rift_scale_and_offset eye_to_source_uv;
};

enum rift_variant
{
	RIFT_VARIANT_DK1,
	RIFT_VARIANT_DK2,
};

#define OCULUS_VR_VID 0x2833

#define OCULUS_DK2_PID 0x0021

/*!
 * Probing function for Oculus Rift devices.
 *
 * @ingroup drv_rift
 * @see xrt_prober_found_func_t
 */
int
rift_found(struct xrt_prober *xp,
           struct xrt_prober_device **devices,
           size_t device_count,
           size_t index,
           cJSON *attached_data,
           struct xrt_device **out_xdev);

/*!
 * A rift HMD device.
 *
 * @implements xrt_device
 */
struct rift_hmd
{
	struct xrt_device base;

	struct xrt_pose pose;

	enum u_logging_level log_level;

	// has built-in mutex so thread safe
	struct m_relation_history *relation_hist;

	struct os_hid_device *hid_dev;
	struct os_thread_helper sensor_thread;
	bool processed_sample_packet;
	uint32_t last_remote_sample_time_us;
	int64_t last_remote_sample_time_ns;

	struct m_imu_3dof fusion;
	struct m_clock_windowed_skew_tracker *clock_tracker;

	int64_t last_keepalive_time;
	enum rift_variant variant;
	struct rift_config_report config;
	struct rift_display_info_report display_info;

	struct rift_lens_distortion *lens_distortions;
	uint16_t num_lens_distortions;
	uint16_t distortion_in_use;

	struct rift_extra_display_info extra_display_info;
};

/// Casting helper function
static inline struct rift_hmd *
rift_hmd(struct xrt_device *xdev)
{
	return (struct rift_hmd *)xdev;
}

struct rift_hmd *
rift_hmd_create(struct os_hid_device *dev, enum rift_variant variant, char *device_name, char *serial_number);

/*!
 * @dir drivers/rift
 *
 * @brief @ref drv_rift files.
 */

#ifdef __cplusplus
}
#endif
