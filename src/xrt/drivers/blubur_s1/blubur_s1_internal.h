// Copyright 2025, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal Blubur S1 driver definitions.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_blubur_s1
 */

#pragma once

#include "util/u_distortion_mesh.h"

#include "blubur_s1_interface.h"


struct blubur_s1_hmd
{
	struct xrt_device base;

	struct u_poly_3k_eye_values poly_3k_values[2];
};
