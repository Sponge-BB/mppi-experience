#!/usr/bin/env python3
"""Exact reversible isolated native-3D switch; never edit production inputs."""
INCLUDE = '#include "native_collision_export.hpp"\n'
MANAGER_MARKER = '    mm_config_->setParam(nh, grid_map_);'
MANAGER_NEW = MANAGER_MARKER + '\n    native_collision_export::initialize(nh, grid_map_, mm_config_);'
BRIDGE_MARKER = '  if(!coupling_geometry::enabled())return true;'
BRIDGE_NEW = '  if(native_collision_export::enabled())return true; // Native 3D gate remains in the original FSM.\n' + BRIDGE_MARKER
GEOMETRY_MARKER = '  if (!coupling_geometry::enabled()) return 0.;'
GEOMETRY_NEW = '  if (native_collision_export::enabled()) return 0.; // Remove only the later projected objective.\n' + GEOMETRY_MARKER
TRANSFORMS = {
    'planner_manager.cpp': (MANAGER_MARKER, MANAGER_NEW),
    'coupling_bridge.hpp': (BRIDGE_MARKER, BRIDGE_NEW),
    'coupling_native_integration.hpp': (GEOMETRY_MARKER, GEOMETRY_NEW),
}


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Expected one exact native collision marker: ' + old)
    return text.replace(old, new, 1)


def transform(text, kind):
    old, new = TRANSFORMS[kind]
    if INCLUDE in text:
        raise ValueError('Native collision patch already present')
    return INCLUDE + replace_once(text, old, new)


def project_off(text, kind):
    old, new = TRANSFORMS[kind]
    return replace_once(replace_once(text, new, old), INCLUDE, '')
