// Copyright 2020-2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Generated bindings helpers header.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup xrt_api
 */

#pragma once
#include <stddef.h>
#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

const char *
xrt_input_name_string(enum xrt_input_name input);

enum xrt_input_name
xrt_input_name_enum(const char *input);

const char *
xrt_output_name_string(enum xrt_output_name output);

enum xrt_output_name
xrt_output_name_enum(const char *output);

// clang-format on"
#ifdef __cplusplus
}
#endif
