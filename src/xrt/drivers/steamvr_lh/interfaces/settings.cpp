// Copyright 2023, Shawn Wallace
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief OpenVR IVRSettings interface implementation.
 * @author Shawn Wallace <yungwallace@live.com>
 * @ingroup drv_steamvr_lh
 */

#include <openvr_driver.h>
#include <optional>
#include <cstring>
#include <string_view>

#include "settings.hpp"
#include "context.hpp"
#include "../device.hpp"
#include "util/u_json.hpp"

using namespace std::string_view_literals;

// Default to 100% brightness.
DEBUG_GET_ONCE_FLOAT_OPTION(lh_default_brightness, "LH_DEFAULT_BRIGHTNESS", 1.0)

using xrt::auxiliary::util::json::JSONNode;

Settings::Settings(const std::string &steam_install, const std::string &steamvr_install, Context *context)
    : steamvr_settings(JSONNode::loadFromFile(steam_install + "/config/steamvr.vrsettings")),
      driver_defaults(
          JSONNode::loadFromFile(steamvr_install + "/drivers/lighthouse/resources/settings/default.vrsettings")),
      context{context}
{
	analog_gain = debug_get_float_option_lh_default_brightness();
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters)
const char *
Settings::GetSettingsErrorNameFromEnum(vr::EVRSettingsError eError)
{
	return nullptr;
}

void
Settings::SetBool(const char *pchSection, const char *pchSettingsKey, bool bValue, vr::EVRSettingsError *peError)
{}

void
Settings::SetInt32(const char *pchSection, const char *pchSettingsKey, int32_t nValue, vr::EVRSettingsError *peError)
{}

void
Settings::SetFloat(const char *pchSection, const char *pchSettingsKey, float flValue, vr::EVRSettingsError *peError)
{
        if (peError != nullptr) {
                *peError = vr::VRSettingsError_None;
        }

        if (pchSection == std::string_view{vr::k_pch_SteamVR_Section} && pchSettingsKey == "analogGain"sv) {
                analog_gain = flValue;

                if (!analog_gain_update_from_device && context != nullptr && context->hmd != nullptr) {
                        context->hmd->apply_analog_gain(flValue);
                }

                if (!analog_gain_update_from_device && context != nullptr) {
                        context->add_vendor_event(vr::VREvent_SteamVRSectionSettingChanged);
                }
        }
}

void
Settings::SetString(const char *pchSection,
                    const char *pchSettingsKey,
                    const char *pchValue,
                    vr::EVRSettingsError *peError)
{}

bool
Settings::GetBool(const char *pchSection, const char *pchSettingsKey, vr::EVRSettingsError *peError)
{
	return false;
}

int32_t
Settings::GetInt32(const char *pchSection, const char *pchSettingsKey, vr::EVRSettingsError *peError)
{
	return 0;
}

float
Settings::GetFloat(const char *pchSection, const char *pchSettingsKey, vr::EVRSettingsError *peError)
{
        if (!strcmp(pchSection, vr::k_pch_SteamVR_Section)) {
                if (!strcmp(pchSettingsKey, "analogGain")) {
                        if (context != nullptr && context->hmd != nullptr) {
                                float brightness = 0.0f;
                                if (context->hmd->get_brightness(&brightness) == XRT_SUCCESS) {
                                        analog_gain = HmdDevice::brightness_to_analog_gain(brightness);
                                }
                        }
                        if (peError != nullptr) {
                                *peError = vr::VRSettingsError_None;
                        }
                        return analog_gain;
                }
                if (!strcmp(pchSettingsKey, "ipd")) {
                        if (peError != nullptr) {
                                *peError = vr::VRSettingsError_None;
                        }
                        if (context != nullptr && context->hmd != nullptr) {
                                return context->hmd->get_ipd();
                        }
                        return 0.f;
                }
        }
        return 0.0;
}

// Driver requires a few string settings to initialize properly
void
Settings::GetString(const char *pchSection,
                    const char *pchSettingsKey,
                    char *pchValue,
                    uint32_t unValueLen,
                    vr::EVRSettingsError *peError)
{
	if (peError)
		*peError = vr::VRSettingsError_None;

	auto get_string = [pchSection, pchSettingsKey](const JSONNode &root) -> std::optional<std::string> {
		JSONNode section = root[pchSection];
		if (!section.isValid())
			return std::nullopt;

		JSONNode value = section[pchSettingsKey];
		if (!value.isValid() || !value.isString())
			return std::nullopt;

		return std::optional(value.asString());
	};

	std::optional value = get_string(driver_defaults);
	if (!value.has_value())
		value = get_string(steamvr_settings);

	if (value.has_value()) {
		if (unValueLen > value->size())
			std::strncpy(pchValue, value->c_str(), value->size() + 1);
	} else if (peError)
		*peError = vr::VRSettingsError_ReadFailed;
}

void
Settings::RemoveSection(const char *pchSection, vr::EVRSettingsError *peError)
{}

void
Settings::RemoveKeyInSection(const char *pchSection, const char *pchSettingsKey, vr::EVRSettingsError *peError)
{}

void
Settings::sync_analog_gain_from_device(float new_analog_gain, bool notify_context)
{
        bool previous_state = analog_gain_update_from_device;
        analog_gain_update_from_device = true;
        analog_gain = new_analog_gain;

        if (notify_context && context != nullptr) {
                context->add_vendor_event(vr::VREvent_SteamVRSectionSettingChanged);
        }

        analog_gain_update_from_device = previous_state;
}
// NOLINTEND(bugprone-easily-swappable-parameters)
