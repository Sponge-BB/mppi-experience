#!/usr/bin/env python3
"""Small file-only tests: no ROS, model rollout or compiler invocation."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import prepare_sphere_timing_build as prepare


class SphereTimingBuild(unittest.TestCase):
    def fixture(self, root):
        parent = root / 'parent'
        (parent / 'source').mkdir(parents=True)
        (parent / 'source/shared_collision.hpp').write_text('shared')
        (parent / 'source/model.hpp').write_text('frozen model')
        (parent / 'build').mkdir()
        native = {}
        for name in prepare.NATIVE:
            path = parent / 'build' / name
            path.write_text('binary:' + name)
            path.chmod(0o755)
            native[name] = prepare.sha(path)
        library = parent / 'build/libonline_mppi.so'
        library.write_text('frozen library')
        archives = {}
        for name in prepare.ARCHIVES:
            path = parent / 'build' / name
            path.write_text('archive:' + name)
            archives[name] = prepare.sha(path)
        (parent / 'build/not_needed.cpp.o').write_text('object not copied')
        before, deployed_sources = {}, {}
        for i in range(8):
            name = 'src/part%d.cpp' % i
            path = root / name
            path.parent.mkdir(exist_ok=True)
            path.write_text('old%d' % i)
            before[name] = prepare.sha(path)
            if i < 7:
                path.write_text('deployed%d' % i)
                deployed_sources[name] = prepare.sha(path)
        sphere = root / 'src/spheres.yaml'
        sphere.write_text('47 spheres')
        deployed_sources['src/spheres.yaml'] = prepare.sha(sphere)
        deployed = root / 'deployed.json'
        deployed.write_text(json.dumps({'files_sha256': deployed_sources}))
        recipe = {key: {} for key in ('inherited_flags_sha256', 'inherited_link_sha256',
            'reused_link_archives_sha256', 'unchanged_source_inputs_sha256', 'frozen_header_inputs_sha256')}
        manifest = dict(production_sources_sha256=before, prepared_only=False,
            urdf_spheres_b_experiment=True, coupling_objective=True, dual_timescale_probe=True,
            native_binaries_sha256=native, mppi_library_sha256=prepare.sha(library),
            isolated_source_sha256=prepare.source_inventory(parent / 'source'),
            urdf_spheres_b_archive_sha256=archives, urdf_spheres_b_recipe=recipe,
            shared_collision_source_sha256=prepare.sha(parent / 'source/shared_collision.hpp'),
            urdf_spheres_b_geometry={'fixed_structure_spheres': 47, 'moving_arm_spheres': 20})
        (parent / 'build_manifest.json').write_text(json.dumps(manifest))
        return parent, deployed, manifest

    def test_copy_preserves_all_runtime_and_archive_bytes_no_objects(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parent, deployed, before = self.fixture(root)
            parent_sha = prepare.sha(parent / 'build_manifest.json')
            new, out = prepare.prepare(root / 'new', parent, root, deployed)
            self.assertEqual(parent_sha, prepare.sha(parent / 'build_manifest.json'))
            self.assertEqual(new['isolated_source_sha256'], before['isolated_source_sha256'])
            self.assertEqual(set(p.name for p in (out / 'build').iterdir()),
                set(prepare.NATIVE) | set(prepare.ARCHIVES) | {'libonline_mppi.so'})
            self.assertTrue(new['binary_unchanged_from_parent'])
            self.assertFalse(new['compiled_in_this_preparation'])
            self.assertEqual(new['native_binaries_sha256'], before['native_binaries_sha256'])
            self.assertEqual(new['sphere_timing_production_rebaseline']['parent_production_sources_sha256'],
                before['production_sources_sha256'])
            self.assertEqual(len(new['sphere_timing_production_rebaseline']['applied_source_replacements']), 7)
            self.assertEqual(set(new['production_sources_sha256']), set(before['production_sources_sha256']))
            self.assertTrue((out / 'build' / prepare.NATIVE[0]).stat().st_mode & 0o111)
            with self.assertRaises(FileExistsError):
                prepare.prepare(out, parent, root, deployed)

    def test_unrecorded_production_drift_is_rejected_before_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parent, deployed, _ = self.fixture(root)
            (root / 'src/part7.cpp').write_text('unrecorded change')
            with self.assertRaisesRegex(ValueError, 'Unrecorded production change'):
                prepare.prepare(root / 'new', parent, root, deployed)
            self.assertFalse((root / 'new').exists())

    def test_recorded_deployment_must_still_match_current_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parent, deployed, _ = self.fixture(root)
            (root / 'src/part0.cpp').write_text('another change')
            with self.assertRaises(ValueError):
                prepare.prepare(root / 'new', parent, root, deployed)
            (root / 'src/spheres.yaml').write_text('changed untracked yaml')
            with self.assertRaises(ValueError):
                prepare.prepare(root / 'new', parent, root, deployed)

    def test_source_extra_or_missing_archive_or_different_runtime_rejected(self):
        for change in ('source', 'archive', 'runtime'):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                parent, deployed, _ = self.fixture(root)
                path = {'source': parent / 'source/extra.hpp',
                    'archive': parent / 'build/libexp_model.a',
                    'runtime': parent / 'build/libonline_mppi.so'}[change]
                path.write_text('changed')
                with self.assertRaises(ValueError):
                    prepare.prepare(root / 'new', parent, root, deployed)
                self.assertFalse((root / 'new').exists())


if __name__ == '__main__':
    unittest.main()
