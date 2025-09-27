// Copyright 2025, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of Blubur S1 HMD driver.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_blubur_s1
 */

#include "math/m_api.h"
#include "math/m_mathinclude.h"

#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_debug.h"

#include "blubur_s1_interface.h"
#include "blubur_s1_internal.h"


DEBUG_GET_ONCE_BOOL_OPTION(blubur_s1_test_distortion, "BLUBUR_S1_TEST_DISTORTION", false)

#define VIEW_SIZE (1440)
#define PANEL_WIDTH (VIEW_SIZE * 2)

static struct blubur_s1_hmd *
blubur_s1_hmd(struct xrt_device *xdev)
{
	return (struct blubur_s1_hmd *)xdev;
}

static void
blubur_s1_hmd_destroy(struct xrt_device *xdev)
{
	struct blubur_s1_hmd *hmd = blubur_s1_hmd(xdev);
	free(hmd);
}

static xrt_result_t
blubur_s1_hmd_compute_poly_3k_distortion(
    struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result)
{
	struct blubur_s1_hmd *hmd = blubur_s1_hmd(xdev);

	struct u_poly_3k_eye_values *values = &hmd->poly_3k_values[view];
	u_compute_distortion_poly_3k(values, view, u, v, out_result);

	return XRT_SUCCESS;
}

static xrt_result_t
blubur_s1_hmd_compute_test_distortion(
    struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result)
{
	float x = u * 2.0f - 1.0f;
	float y = v * 2.0f - 1.0f;

	float r2 = x * x + y * y;
	float r = sqrtf(r2);
	float r3 = r2 * r;
	float r4 = r2 * r2;
	float r5 = r4 * r;

	float radial = (0.5978f * r5) - (0.7257f * r4) + (0.504f * r3) - (0.0833f * r2) + (0.709f * r) - 0.00006f;

	struct xrt_vec2 result = {
	    .x = (x * radial) / 2.0f + 0.5f,
	    .y = (y * radial) / 2.0f + 0.5f,
	};
	out_result->r = result;
	out_result->g = result;
	out_result->b = result;

	return XRT_SUCCESS;
}

static xrt_result_t
blubur_s1_hmd_update_inputs(struct xrt_device *xdev)
{
	return XRT_SUCCESS;
}

static xrt_result_t
blubur_s1_hmd_get_tracked_pose(struct xrt_device *xdev,
                               enum xrt_input_name name,
                               int64_t at_timestamp_ns,
                               struct xrt_space_relation *out_relation)
{
	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	// TODO: track pose
	*out_relation = (struct xrt_space_relation){
	    .relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT,
	    .pose =
	        {
	            .orientation = XRT_QUAT_IDENTITY,
	        },
	};

	return XRT_SUCCESS;
}

static xrt_result_t
blubur_s1_hmd_get_presence(struct xrt_device *xdev, bool *presence)
{
	// TODO: read the presence sensor from the device
	*presence = true;

	return XRT_SUCCESS;
}

static xrt_result_t
blubur_s1_hmd_get_view_poses(struct xrt_device *xdev,
                             const struct xrt_vec3 *default_eye_relation,
                             int64_t at_timestamp_ns,
                             uint32_t view_count,
                             struct xrt_space_relation *out_head_relation,
                             struct xrt_fov *out_fovs,
                             struct xrt_pose *out_poses)
{
	return u_device_get_view_poses(xdev, default_eye_relation, at_timestamp_ns, view_count, out_head_relation,
	                               out_fovs, out_poses);
}

static void
blubur_s1_hmd_fill_in_poly_3k(struct blubur_s1_hmd *hmd)
{
	hmd->poly_3k_values[0] = (struct u_poly_3k_eye_values){
	    .channels =
	        {
	            {
	                .display_size = {PANEL_WIDTH, VIEW_SIZE},
	                .eye_center = {711.37015431841485f, 702.64004980572099f},
	                .k =
	                    {
	                        2.4622190410034843e-007f,
	                        1.0691119647014047e-012f,
	                        6.9872433537257567e-019f,
	                    },
	            },
	            {
	                .display_size = {PANEL_WIDTH, VIEW_SIZE},
	                .eye_center = {710.34756994635097f, 702.30352808724865f},
	                .k =
	                    {
	                        3.3081468849915169e-007f,
	                        6.6872723393907828e-013f,
	                        1.5518253834715642e-018f,
	                    },
	            },
	            {
	                .display_size = {PANEL_WIDTH, VIEW_SIZE},
	                .eye_center = {709.19922270098721f, 702.42895617576141f},
	                .k =
	                    {
	                        4.6306924021839207e-007f,
	                        1.5032174824131911e-013f,
	                        2.6240474534705725e-018f,
	                    },
	            },
	        },
	};
	// NOTE: these distortion values appear to exhibit the Y offset bug that some WMR headsets do, worked around it
	//       by copying eye center Y to the other eye
	hmd->poly_3k_values[1] = (struct u_poly_3k_eye_values){
	    .channels =
	        {
	            {
	                .display_size = {PANEL_WIDTH, VIEW_SIZE},
	                .eye_center = {2166.0195141711984f,
	                               hmd->poly_3k_values->channels[0].eye_center.y /* 693.80762487779759f */},
	                .k =
	                    {
	                        1.6848296693566205e-007f,
	                        1.1446999540490656e-012f,
	                        1.8794325973106313e-019f,
	                    },
	            },
	            {
	                .display_size = {PANEL_WIDTH, VIEW_SIZE},
	                .eye_center = {2164.9567320272263f,
	                               hmd->poly_3k_values->channels[1].eye_center.y /* 693.8666328641682f */},
	                .k =
	                    {
	                        2.2979021408214227e-007f,
	                        9.2094643470416607e-013f,
	                        6.8614927296300735e-019f,
	                    },
	            },
	            {
	                .display_size = {PANEL_WIDTH, VIEW_SIZE},
	                .eye_center = {2164.0315727658904f,
	                               hmd->poly_3k_values->channels[2].eye_center.y /* 693.45351818980896f */},
	                .k =
	                    {
	                        3.1993667496208384e-007f,
	                        6.1930456677642785e-013f,
	                        1.2848584929803272e-018f,
	                    },
	            },
	        },
	};

	struct xrt_matrix_3x3 affine_xform[2] = {
	    {
	        .v =
	            {
	                886.745f, 0.205964f, 710.326f, //
	                0, 886.899f, 706.657f,         //
	                0, 0, 1,                       //
	            },
	    },
	    {
	        .v =
	            {
	                880.317f, 0.277553f, 2163.58f, //
	                0, 879.669f, 698.35f,          //
	                0, 0, 1,                       //
	            },
	    },
	};

	for (int i = 0; i < 2; i++) {
		math_matrix_3x3_inverse(&affine_xform[i], &hmd->poly_3k_values[i].inv_affine_xform);

		u_compute_distortion_bounds_poly_3k(
		    &hmd->poly_3k_values[i].inv_affine_xform, hmd->poly_3k_values[i].channels, i,
		    &hmd->base.hmd->distortion.fov[i], &hmd->poly_3k_values[i].tex_x_range,
		    &hmd->poly_3k_values[i].tex_y_range);
	}
}

struct blubur_s1_hmd *
blubur_s1_hmd_create(struct os_hid_device *dev, const char *serial)
{
	struct blubur_s1_hmd *hmd =
	    U_DEVICE_ALLOCATE(struct blubur_s1_hmd, U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE, 1, 0);
	if (hmd == NULL) {
		return NULL;
	}

	hmd->base.destroy = blubur_s1_hmd_destroy;
	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;

	hmd->base.hmd->screens[0].w_pixels = PANEL_WIDTH;
	hmd->base.hmd->screens[0].h_pixels = VIEW_SIZE;
	hmd->base.hmd->screens[0].nominal_frame_interval_ns = 1000000000LLU / 120; // 120hz

	hmd->base.hmd->view_count = 2;
	hmd->base.hmd->views[0] = (struct xrt_view){
	    .viewport =
	        {
	            .x_pixels = 0,
	            .y_pixels = 0,
	            .w_pixels = VIEW_SIZE,
	            .h_pixels = VIEW_SIZE,
	        },
	    .display =
	        {
	            .w_pixels = VIEW_SIZE,
	            .h_pixels = VIEW_SIZE,
	        },
	    .rot = u_device_rotation_ident,
	};
	hmd->base.hmd->views[1] = (struct xrt_view){
	    .viewport =
	        {
	            .x_pixels = VIEW_SIZE,
	            .y_pixels = 0,
	            .w_pixels = VIEW_SIZE,
	            .h_pixels = VIEW_SIZE,
	        },
	    .display =
	        {
	            .w_pixels = VIEW_SIZE,
	            .h_pixels = VIEW_SIZE,
	        },
	    .rot = u_device_rotation_ident,
	};

	hmd->base.hmd->blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = 1;

	blubur_s1_hmd_fill_in_poly_3k(hmd);

	hmd->base.hmd->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	hmd->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
	hmd->base.compute_distortion = debug_get_bool_option_blubur_s1_test_distortion()
	                                   ? blubur_s1_hmd_compute_test_distortion
	                                   : blubur_s1_hmd_compute_poly_3k_distortion;
	u_distortion_mesh_fill_in_compute(&hmd->base);

	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "Blubur S1");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "%s", serial);

	hmd->base.supported.position_tracking = true;
	hmd->base.supported.presence = true;

	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;

	hmd->base.update_inputs = blubur_s1_hmd_update_inputs;
	hmd->base.get_tracked_pose = blubur_s1_hmd_get_tracked_pose;
	hmd->base.get_presence = blubur_s1_hmd_get_presence;
	hmd->base.get_view_poses = blubur_s1_hmd_get_view_poses;

	return hmd;
}
