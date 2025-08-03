#!/usr/bin/env python3
# Copyright 2020-2025, Collabora, Ltd.
# Copyright 2024-2025, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0
"""Generate OpenXR binding code using the auxiliary bindings scripts."""

from bindings import *

import argparse
from operator import attrgetter
from string import Template

def get_verify_switch_body(dict_of_lists, profile, profile_name, tab_char):
    """Generate function to check if a string is in a set of strings.
    Input is a dict where keys are length and
    the values are lists of strings of that length. And a suffix if any.
    Returns a list of lines."""
    ret = [ f"{tab_char}\tswitch (length) {{" ]
    for length in sorted(dict_of_lists.keys()):
        ret += [
            f'{tab_char}\tcase {str(length)}:',
        ]

        for i, path in enumerate(sorted(dict_of_lists[length])):
            if_or_else_if = 'if' if i == 0 else '} else if'
            ret += [
                f'{tab_char}\t\t{if_or_else_if} (strcmp(str, "{path}") == 0) {{',
                f'{tab_char}\t\t\treturn true;',
             ]
        ret += [
            f'{tab_char}\t\t}}',
            f'{tab_char}\t\tbreak;',
        ]
    ret += [
        f'{tab_char}\tdefault: break;',
        f'{tab_char}\t}}',
    ]
    return ret

def get_verify_func_switch(dict_of_lists, profile, profile_name, availability):
    """Generate function to check if a string is in a set of strings.
    Input is a dict where keys are length and
    the values are lists of strings of that length. And a suffix if any.
    Returns a list of lines."""
    if len(dict_of_lists) == 0:
        return ''

    ret = [
        f"\t// generated from: {profile_name}"
    ]

    # Example: pico neo 3 can be enabled by either enabling XR_BD_controller_interaction ext or using OpenXR 1.1+.
    # Disabling OXR_HAVE_BD_controller_interaction should NOT remove pico neo from OpenXR 1.1+ (it makes "exts->BD_controller_interaction" invalid C code).
    # Therefore separate code blocks for ext and version checks generated to avoid ifdef hell.
    feature_sets = sorted(availability.feature_sets, key=FeatureSet.as_tuple)
    for feature_set in feature_sets:
        requires_version = is_valid_version(feature_set.required_version)
        requires_extensions = bool(feature_set.required_extensions)

        tab_char = ''
        closing = []

        if requires_version:
            tab_char += '\t'
            ret += [
                f'{tab_char}if (openxr_version >= XR_MAKE_VERSION({feature_set.required_version["major"]}, {feature_set.required_version["minor"]}, 0)) {{',
            ]
            closing.append(f'{tab_char}}}\n')

        if requires_extensions:
            tab_char += '\t'
            exts = sorted(feature_set.required_extensions)
            ext_defines = ' && '.join(f'defined(OXR_HAVE_{ext})' for ext in exts)
            ret += [
                f'#if {ext_defines}',
                f'{tab_char}if ('+' && '.join(f'exts->{ext}' for ext in exts)+') {',
            ]
            closing.append(f'{tab_char}}}\n#endif // {ext_defines}\n\n')

        ret += get_verify_switch_body(dict_of_lists, profile, profile_name, tab_char)

        ret += reversed(closing)

    return ret

def get_verify_func_body(profile, dict_name, availability):
    """returns a list of lines"""
    ret = []
    if profile is None or dict_name is None or len(dict_name) == 0:
        return ret
    ret += get_verify_func_switch(getattr(
        profile, dict_name), profile, profile.name, availability)
    if profile.parent_profiles is None:
        return ret
    for pp in sorted(profile.parent_profiles, key=attrgetter("name")):
        ret += get_verify_func_body(pp, dict_name, availability.intersection(pp.availability()))

    return ret


def get_verify_func(profile, dict_name, suffix):
    """returns a list of lines"""
    name = f"oxr_verify_{profile.validation_func_name}{suffix}"

    ret = [
        'bool',
        '{name}(const struct oxr_extension_status *exts, XrVersion openxr_version, const char *str, size_t length)'.format(name=name),
        '{',

        *get_verify_func_body(profile, dict_name, profile.availability()),

        '\treturn false;',
        '}',
        ''
    ]

    return ret

def get_verify_functions(profile):
    """returns a list of lines"""
    ret = [
        *get_verify_func(profile, "subpaths_by_length", "_subpath"),
        *get_verify_func(profile, "dpad_paths_by_length", "_dpad_path"),
        *get_verify_func(profile, "dpad_emulators_by_length", "_dpad_emulator"),

        'void',
        f'oxr_verify_{profile.validation_func_name}_ext(const struct oxr_extension_status *extensions, XrVersion openxr_version, bool *out_supported, bool *out_enabled)',
        '{',
        '',
    ]

    is_promoted = is_valid_version(profile.openxr_version_promoted)
    if is_promoted:
        ret += [
            f'\tif (openxr_version >= XR_MAKE_VERSION({profile.openxr_version_promoted["major"]}, {profile.openxr_version_promoted["minor"]}, 0)) {{',
            '\t\t*out_supported = true;',
            '\t\t*out_enabled = true;',
            '\t\treturn;',
            '\t}',
            '',
        ]


    if profile.extension_name is not None:
        ret += [
            f'#ifdef OXR_HAVE_{profile.extension_name}',
            '\t*out_supported = true;',
            f'\t*out_enabled = extensions->{profile.extension_name};',
            '#else',
            '\t*out_supported = false;',
            '\t*out_enabled = false;',
            f'#endif // OXR_HAVE_{profile.extension_name}',
            '}',
            '',
        ]

    else:
        ret += [
            '\t*out_supported = true;',
            '\t*out_enabled = true;',
            '}',
            '',
        ]

    return ret


def generate_bindings_c(file, b):
    """Generate the file to verify subpaths on a interaction profile."""
    f = open(file, "w")

    wl(f,
        header.format(brief='Generated bindings data', group='oxr_main'),
        '#include "oxr_bindings/b_oxr_generated_bindings.h"',
        '#include <string.h>',
        '#include "oxr_objects.h"',
        '',
        '// clang-format off',
        '',

        # unpack list comprehension that put the list of returned lines from the inner loop into a new list
        *[verify_function_lines
            for profile in b.profiles
                for verify_function_lines in get_verify_functions(profile)
        ],

        f'\n\nstruct profile_template profile_templates[{len(b.profiles)}] = {{ // array of profile_template',
    )


    for profile in b.profiles:
        hw_name = str(profile.name.split("/")[-1])
        vendor_name = str(profile.name.split("/")[-2])
        fname = vendor_name + "_" + hw_name + "_profile.json"
        controller_type = "monado_" + vendor_name + "_" + hw_name

        binding_count = len(profile.components)
        wl(f,
            f'\t{{ // profile_template',
            f'\t\t.name = {profile.monado_device_enum},',
            f'\t\t.path = "{profile.name}",',
            f'\t\t.localized_name = "{profile.localized_name}",',
            f'\t\t.steamvr_input_profile_path = "{fname}",',
            f'\t\t.steamvr_controller_type = "{controller_type}",',
            f'\t\t.binding_count = {binding_count},',
            f'\t\t.bindings = (struct binding_template[]){{ // array of binding_template',
        )

        component: Component
        for idx, component in enumerate(profile.components):

            # @todo Doesn't handle pose yet.
            steamvr_path = component.steamvr_path
            if component.component_name in ["click", "touch", "force", "value", "proximity"]:
                steamvr_path += "/" + component.component_name

            wl(f,
                f'\t\t\t{{ // binding_template {idx}',
                f'\t\t\t\t.subaction_path = "{component.subaction_path}",',
                f'\t\t\t\t.steamvr_path = "{steamvr_path}",',
                f'\t\t\t\t.localized_name = "{component.subpath_localized_name}",',
                '',
                '\t\t\t\t.paths = { // array of paths',

                '\n'.join([f'\t\t\t\t\t"{path}",' for path in component.get_full_openxr_paths() ]),

                '\t\t\t\t\tNULL,',
                '\t\t\t\t}, // /array of paths',
            )

            # print("component", component.__dict__)

            component_str = component.component_name

            # controllers can have input that we don't have bindings for
            if component.monado_binding:
                monado_binding = component.monado_binding

                # Input, dpad_activate and output default to 0.
                # If a binding specifies an actual binding value for them in json, those get overridden.
                input_value = '0'
                dpad_activate_value = '0'
                output_value = '0'

                if component.is_input() and monado_binding is not None:
                    input_value = monado_binding

                if component.has_dpad_emulation() and "activate" in component.dpad_emulation:
                    activate_component = find_component_in_list_by_name(
                        component.dpad_emulation["activate"], profile.components,
                        subaction_path=component.subaction_path,
                        identifier_json_path=component.identifier_json_path)
                    dpad_activate_value = activate_component.monado_binding

                if component.is_output() and monado_binding is not None:
                    output_value = monado_binding

                wl(f,
                    f'\t\t\t\t.input = {input_value},',
                    f'\t\t\t\t.dpad_activate = {dpad_activate_value},',
                    f'\t\t\t\t.output = {output_value},',
                )

            wl(f, f'\t\t\t}}, // /binding_template {idx}')

        wl(f, '\t\t}, // /array of binding_template')

        dpads = []
        for idx, identifier in enumerate(profile.identifiers):
            if identifier.dpad:
                dpads.append(identifier)

#        for identifier in dpads:
#            print(identifier.path, identifier.dpad_position_component)

        dpad_count = len(dpads)
        f.write(f'\t\t.dpad_count = {dpad_count},\n')
        if len(dpads) == 0:
            f.write(f'\t\t.dpads = NULL,\n')
        else:
            f.write(
                f'\t\t.dpads = (struct dpad_emulation[]){{ // array of dpad_emulation\n')
            for idx, identifier in enumerate(dpads):
                f.write('\t\t\t{\n')
                f.write(f'\t\t\t\t.subaction_path = "{identifier.subaction_path}",\n')
                f.write('\t\t\t\t.paths = {\n')
                for path in identifier.dpad.paths:
                    f.write(f'\t\t\t\t\t"{path}",\n')
                f.write('\t\t\t\t},\n')
                f.write(f'\t\t\t\t.position = {identifier.dpad.position_component.monado_binding},\n')
                if identifier.dpad.activate_component:
                    f.write(f'\t\t\t\t.activate = {identifier.dpad.activate_component.monado_binding},\n')
                else:
                    f.write(f'\t\t\t\t.activate = 0')

                f.write('\t\t\t},\n')
            f.write('\t\t}, // /array of dpad_emulation\n')

        f.write(f'\t\t.openxr_version.promoted.major = {profile.openxr_version_promoted["major"]},\n')
        f.write(f'\t\t.openxr_version.promoted.minor = {profile.openxr_version_promoted["minor"]},\n')

        fn_prefixes = ["subpath", "dpad_path", "dpad_emulator"]
        for prefix in fn_prefixes:
            f.write(f'\t\t.{prefix}_fn = oxr_verify_{profile.validation_func_name}_{prefix},\n')
        f.write(f'\t\t.ext_verify_fn = oxr_verify_{profile.validation_func_name}_ext,\n')
        if profile.extension_name is None:
            f.write(f'\t\t.extension_name = NULL,\n')
        else:
            f.write(f'\t\t.extension_name = "{profile.extension_name}",\n')
        f.write('\t}, // /profile_template\n')

    f.write('}; // /array of profile_template\n\n')

    f.write("\n// clang-format on\n")

    f.close()


H_TEMPLATE = Template("""$header

#pragma once

#include <stddef.h>
#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint64_t XrPath; // OpenXR typedef'
typedef uint64_t XrVersion; // OpenXR typedef'

struct oxr_extension_status;

#define OXR_BINDINGS_PROFILE_TEMPLATE_COUNT $template_count

// clang-format off

$verify_protos

#define PATHS_PER_BINDING_TEMPLATE 16

enum oxr_dpad_binding_point
{
\tOXR_DPAD_BINDING_POINT_NONE,
\tOXR_DPAD_BINDING_POINT_UP,
\tOXR_DPAD_BINDING_POINT_DOWN,
\tOXR_DPAD_BINDING_POINT_LEFT,
\tOXR_DPAD_BINDING_POINT_RIGHT,
};

struct dpad_emulation
{
\tconst char *subaction_path;
\tconst char *paths[PATHS_PER_BINDING_TEMPLATE];
\tenum xrt_input_name position;
\tenum xrt_input_name activate; // Can be zero
};

struct binding_template
{
\tconst char *subaction_path;
\tconst char *steamvr_path;
\tconst char *localized_name;
\tconst char *paths[PATHS_PER_BINDING_TEMPLATE];
\tenum xrt_input_name input;
\tenum xrt_input_name dpad_activate;
\tenum xrt_output_name output;
};

typedef bool (*path_verify_fn_t)(const struct oxr_extension_status *extensions, XrVersion openxr_version, const char *, size_t);
typedef void (*ext_verify_fn_t)(const struct oxr_extension_status *extensions, XrVersion openxr_version, bool *out_supported, bool *out_enabled);

struct profile_template
{
\tenum xrt_device_name name;
\tconst char *path;
\tconst char *localized_name;
\tconst char *steamvr_input_profile_path;
\tconst char *steamvr_controller_type;
\tstruct binding_template *bindings;
\tsize_t binding_count;
\tstruct dpad_emulation *dpads;
\tsize_t dpad_count;
\tstruct {
\t\tstruct {
\t\t\tuint32_t major;
\t\t\tuint32_t minor;
\t\t} promoted;
\t} openxr_version;
\t//! path_cache is an INVALID value until oxr_instance_create initializes it.
\tXrPath path_cache;

\tpath_verify_fn_t subpath_fn;
\tpath_verify_fn_t dpad_path_fn;
\tpath_verify_fn_t dpad_emulator_fn;
\text_verify_fn_t ext_verify_fn;
\tconst char *extension_name;
};

extern struct profile_template profile_templates[OXR_BINDINGS_PROFILE_TEMPLATE_COUNT];
// clang-format on")
#ifdef __cplusplus
}
#endif
""")

def generate_bindings_h(file, b):
    """Generate header for the verify subpaths functions."""

    verify_protos = []
    fn_prefixes = ["_subpath", "_dpad_path", "_dpad_emulator"]
    for profile in b.profiles:
        for fn_suffix in fn_prefixes:
            verify_protos += [
                'bool',
                f'oxr_verify_{profile.validation_func_name}{fn_suffix}(const struct oxr_extension_status *extensions, XrVersion openxr_major_minor, const char *str, size_t length);',
                '',
            ]
        verify_protos += [
            'void',
            f'oxr_verify_{profile.validation_func_name}_ext(const struct oxr_extension_status *extensions, XrVersion openxr_version, bool *out_supported, bool *out_enabled);',
            '',
        ]

    with open(file, "w") as f:
        filled = H_TEMPLATE.substitute(
            header = header.format(brief='Generated bindings data', group='oxr_main'),
            template_count=len(b.profiles),
            verify_protos='\n'.join(verify_protos)
        )
        f.write(filled)


def main():
    """Handle command line and generate a file."""
    parser = argparse.ArgumentParser(description='OpenXR Bindings generator.')
    parser.add_argument(
        'bindings', help='Bindings file to use')
    parser.add_argument(
        'output', type=str, nargs='+',
        help='Output file, uses the name to choose output type')
    args = parser.parse_args()

    bindings = Bindings.load_and_parse(args.bindings)

    for output in args.output:
        if output.endswith("oxr_generated_bindings.c"):
            generate_bindings_c(output, bindings)
        if output.endswith("oxr_generated_bindings.h"):
            generate_bindings_h(output, bindings)


if __name__ == "__main__":
    main()
