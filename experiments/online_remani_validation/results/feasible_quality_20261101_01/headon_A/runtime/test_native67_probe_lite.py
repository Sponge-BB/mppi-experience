import unittest
from build_native67_probe_lite import BASE, OLD, NEW, transform


class LiteCaptureTests(unittest.TestCase):
    def test_only_frame_serialization_changes(self):
        before=(BASE/'source/poly_traj_optimizer.cpp').read_text();after=transform(before)
        self.assertEqual(after.replace(NEW,OLD,1),before)
        self.assertNotIn('delivery_diag::frameJson(',after)
        self.assertNotIn('snapshotFile(',after)
        for field in ('sequence','stamp','snapshot_payload_dumped'):
            self.assertIn(field,NEW)

    def test_metadata_does_not_access_optional_directory_or_map_payload(self):
        for token in ('directory(', 'snapshotFile(', 'snapshotText(', 'mkdir', 'writeExclusive', '->field'):
            self.assertNotIn(token,NEW)
        after=transform((BASE/'source/poly_traj_optimizer.cpp').read_text())
        self.assertIn('coupling_geometry::activeFrame().snapshot==frozen_frame.snapshot',after)
        self.assertIn('raw==grid_map_->getDynamicObservationSeq()',after)
        self.assertIn('esdf==grid_map_->getESDFObservationSeq()',after)
        self.assertIn('native_dynamic_stable',after)


if __name__=='__main__':unittest.main()
