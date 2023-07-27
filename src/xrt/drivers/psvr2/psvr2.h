// Copyright 2023, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR2 HMD device internals
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_psvr2
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <asm/byteorder.h>

#define PSVR2_SLAM_INTERFACE 3
#define PSVR2_SLAM_ENDPOINT 3

#define PSVR2_CAMERA_INTERFACE 6
#define PSVR2_CAMERA_ENDPOINT 7

#define PSVR2_STATUS_INTERFACE 7
#define PSVR2_STATUS_ENDPOINT 8

#define PSVR2_LD_INTERFACE 8
#define PSVR2_LD_ENDPOINT 9

#define PSVR2_RP_INTERFACE 9
#define PSVR2_RP_ENDPOINT 10

#define PSVR2_VD_INTERFACE 10
#define PSVR2_VD_ENDPOINT 11

struct imu_record
{
	uint32_t vts;
	int16_t accel[3];
	int16_t gyro[3];
	uint16_t dp_frame_cnt;
	uint16_t dp_line_cnt;
	uint16_t imu_ts;
	uint16_t status;
};

struct imu_usb_record
{
	__le32 vts;
	__le16 accel[3];
	__le16 gyro[3];
	__le16 dp_frame_cnt;
	__le16 dp_line_cnt;
	__le16 imu_ts;
	__le16 status;
} __attribute__((packed));

struct status_record_hdr
{
	uint8_t dprx_status;        /* 0 = not ready. 2 = cinematic? and 1 = unknown. HDCP? Other? */
	uint8_t prox_sensor_flag;   /* 0 = not triggered. 1 = triggered? */
	uint8_t passthrough_button; /* 0 = not pressed, 1 = pressed */
	uint8_t empty0[2];
	uint8_t ipd_dial_mm; /* 59 to 72mm */

	uint8_t remainder[26];
} __attribute__((packed));

struct slam_record
{
	uint32_t ts;      // Timestamp
	double pos[3];    // 32-bit floats
	double orient[4]; // Orientation quaternion
	uint8_t remainder[470];
};

struct slam_usb_record
{
	char SLAhdr[3];   // "SLA"
	uint8_t const1;   // Constant 0x01?
	__le32 pkt_size;  // 0x0200 = 512 bytes;
	__le32 ts;        // Timestamp
	__le32 unknown1;  // Unknown. Constant 3?
	__le32 pos[3];    // 32-bit floats
	__le32 orient[4]; // Orientation quaternion
	uint8_t remainder[468];
} __attribute__((packed));

struct sie_ctrl_pkt
{
	__le16 report_id;
	__le16 subcmd;
	__le32 len;
	uint8_t data[512 - 8];
} __attribute__((packed));

enum psvr2_camera_mode
{
	PSVR2_CAMERA_MODE_OFF = 0,
	PSVR2_CAMERA_MODE_1 = 1,
	PSVR2_CAMERA_MODE_10 = 0x10,
};

#ifdef __cplusplus
}
#endif
