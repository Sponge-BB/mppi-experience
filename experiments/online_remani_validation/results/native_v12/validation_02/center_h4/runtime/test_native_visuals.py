#!/usr/bin/env python3
"""Projection, freshness, read-only search export and bounded logger tests."""
import ctypes as ct
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from native_visuals import geometry, world_disks, SearchReplay
from search_recording import SearchRecorder
from test_mppi_v2 import Controller, ptr

HERE=Path(__file__).resolve().parent
FIXTURE=HERE/'results/native_v11/validation_05/static'
BUILD=HERE/'builds/native_v11_08'


class VisualTests(unittest.TestCase):
    def test_geometry_matches_collision_input(self):
        body,meta=geometry(FIXTURE)
        np.testing.assert_array_equal(body,np.loadtxt(FIXTURE/'geometry.csv',delimiter=','))
        self.assertEqual(body.shape,(3,3))
        pose=[1,2,np.pi/2];world=world_disks(body,pose)
        np.testing.assert_allclose(world[:,:2],np.column_stack((1-body[:,1],2+body[:,0])))
        np.testing.assert_array_equal(world[:,2],body[:,2])

    def test_no_point_fallback(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(FileNotFoundError):geometry(Path(d))

    def test_freshness_and_terminal(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'search').mkdir()
            np.savez(p/'search/000000.npz',published=2.,t=1.98,rows=np.empty((0,8)))
            replay=SearchReplay(p)
            self.assertIsNone(replay.at(1.99))
            self.assertIsNotNone(replay.at(2.2))
            self.assertIsNone(replay.at(2.251))
            self.assertIsNone(replay.at(2.,terminal=True))

    def test_real_getter_does_not_change_next_control(self):
        a=Controller(BUILD,FIXTURE);b=Controller(BUILD,FIXTURE)
        x=np.linspace(0,3,301);ref=np.ascontiguousarray(np.column_stack((x,x*0,x*0,x*0+.18)))
        s=np.array([.5,0.,0.,.18,0.]);obs=np.ascontiguousarray([[1.4,-.5,0.,.26,.23,1.]])
        try:
            with tempfile.TemporaryDirectory() as d:
                logger=SearchRecorder(a.lib,a.handle,Path(d))
                for i in range(4):
                    ra=a.step(s,ref,obs);rb=b.step(s,ref,obs)
                    np.testing.assert_array_equal(ra[:7],rb[:7]) # exclude elapsed milliseconds
                    logger.capture(i*.2,i*.2+.02,s,1,0,'test',-1,False)
                logger.close()
                self.assertFalse(logger.stats['errors'])
                self.assertGreater(logger.stats['written'],0)
                replay=SearchReplay(Path(d));snap=replay.at(.02)
                self.assertIsNotNone(snap)
                rows=snap['rows'];self.assertGreater(len(rows),0)
                self.assertTrue(np.isfinite(rows[:,1:4]).all())
                for ident in np.unique(rows[:,0]):
                    group=rows[rows[:,0]==ident]
                    np.testing.assert_array_equal(group[0,2:4],s[:2])
                    self.assertTrue(np.all(np.diff(group[:,1])>0))
        finally:a.close();b.close()

    def test_rejected_input_never_exports_previous_search(self):
        c=Controller(BUILD,FIXTURE)
        try:
            with tempfile.TemporaryDirectory() as d:
                logger=SearchRecorder(c.lib,c.handle,Path(d))
                logger.capture(0,.01,None,0,-1,'waiting',-1,False)
                logger.close()
                snap=SearchReplay(Path(d)).at(.01)
                self.assertEqual(snap['rows'].shape,(0,8))
        finally:c.close()


if __name__=='__main__':unittest.main()
