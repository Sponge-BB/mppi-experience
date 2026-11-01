#!/usr/bin/env python3
"""Exact, reversible terminal-yaw branch experiment; does not write files.

Scope: the retained dynamic-history MPPI branch only.  The switch is read once
at controller construction; unset/anything other than exactly '1' stays OFF.
No weights, thresholds, proposals, safety checks or LegacyMPPI behavior change.
The including translation unit must provide <cstdlib> before this header.
"""
import hashlib
from pathlib import Path


HERE = Path(__file__).resolve().parent
DEFAULT_SOURCE = HERE / 'builds/seed_parameterization_20261025_01/source/mppi_v12.hpp'
BASE_SHA256 = '26d0e64b9473d9efa6e7e91ed5b801c425db42ed8ebfd4a7b280fe81b6d40e74'
SWITCH = 'MPPI_TERMINAL_PHASE_FIX'
PATCHES = (
    (
        '  int d_clear=0;\n',
        '  int d_clear=0;\n'
        '  const bool terminal_phase_fix;\n'
        '  static bool readTerminalPhaseFix(){\n'
        '    const char* value=std::getenv("MPPI_TERMINAL_PHASE_FIX");\n'
        "    return value && value[0]=='1' && value[1]=='\\0';\n"
        '  }\n',
    ),
    (
        '  MPPI(const C&config,int seed):LegacyMPPI(config,seed){}\n',
        '  MPPI(const C&config,int seed):LegacyMPPI(config,seed),\n'
        '    terminal_phase_fix(readTerminalPhaseFix()){}\n',
    ),
    (
        '      else if(!side_pass&&goalCapture(s,sc))target_yaw=sc.path.back().yaw;\n',
        '      // ON: a predicted entry must not suddenly demand final yaw while\n'
        '      // following() proposes an attainable positional connection.\n'
        '      else if(!terminal_phase_fix&&!side_pass&&goalCapture(s,sc))target_yaw=sc.path.back().yaw;\n',
    ),
)


def sha256(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def transform_mppi(text):
    """Accept exactly the attested parent; refuse drift or double application."""
    if sha256(text) != BASE_SHA256:
        raise ValueError('Unattested mppi_v12.hpp parent or patch already applied')
    result = text
    for old, new in PATCHES:
        if result.count(old) != 1:
            raise ValueError('Missing/ambiguous terminal phase patch anchor')
        result = result.replace(old, new, 1)
    return result


def project_off(text):
    """Undo only the exact transform, then attest every recovered parent byte."""
    result = text
    for old, new in reversed(PATCHES):
        if result.count(new) != 1:
            raise ValueError('Missing/ambiguous patched terminal phase anchor')
        result = result.replace(new, old, 1)
    if sha256(result) != BASE_SHA256:
        raise ValueError('OFF projection does not match attested parent')
    return result


def self_test(source=DEFAULT_SOURCE):
    original = Path(source).read_text()
    patched = transform_mppi(original)
    if project_off(patched) != original:
        raise AssertionError('Terminal phase OFF projection is not byte-exact')
    return dict(base_sha256=BASE_SHA256, patched_sha256=sha256(patched),
                exact_off_projection=True, switch=SWITCH, default_enabled=False,
                constructor_only_switch_read=True,
                scope='dynamic-history predicted capture final-yaw branch only',
                weights_thresholds_candidates_safety_legacy_unchanged=True,
                production_modified=False, ros_started=False)


if __name__ == '__main__':
    import json
    print(json.dumps(self_test(), indent=2))
