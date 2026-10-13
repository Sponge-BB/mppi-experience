#!/usr/bin/env python3
"""Memory-only tests of the actual production Tracker, including no-network guard."""
import ctypes as ct
import json
from pathlib import Path
import socket
import sys
import unittest
from unittest.mock import patch
from run import TrackerAdapter, Stamp, MemoryPublisher

PARAMS={}


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.network=patch.object(socket,'socket',side_effect=AssertionError('network is forbidden in this experiment'))
        self.network.start()
        self.adapter=TrackerAdapter(PARAMS)
        self.inputs=(ct.c_double*10)(0,0,0,0,0,.07,0,0,.15,0)
        self.outputs=(ct.c_double*3)()
        self.adapter.call(1,0,self.inputs,self.outputs)

    def tearDown(self):
        self.network.stop()
        self.assertFalse(self.adapter.errors)

    def test_forward(self):
        self.adapter.call(0,.05,self.inputs,self.outputs)
        self.assertGreater(self.outputs[0],0)
        self.assertLessEqual(self.outputs[0],PARAMS['/real_base_tracker/tracker_max_linear_accel']*.05+1e-9)
        self.assertIsInstance(self.adapter.tracker.cmd_pub,MemoryPublisher)

    def test_reverse_not_blocked(self):
        self.inputs[5]=-.07;self.inputs[8]=-.15
        self.adapter.call(0,.05,self.inputs,self.outputs)
        self.assertLess(self.outputs[0],0)
        self.assertEqual(self.outputs[2],0)

    def test_in_place_alignment(self):
        self.inputs[5]=0;self.inputs[7]=1;self.inputs[8]=0
        self.adapter.call(0,.05,self.inputs,self.outputs)
        self.assertEqual(self.outputs[0],0)
        self.assertGreater(self.outputs[1],0)
        self.assertEqual(self.outputs[2],0)

    def test_stale_odom_stops(self):
        self.adapter.call(0,.05,self.inputs,self.outputs)
        Stamp.clock+=1
        self.adapter.tracker.control_loop(None)
        self.assertEqual(self.adapter.tracker.cmd_pub.last.linear.x,0)
        self.assertTrue(self.adapter.tracker.tracking_fault)

    def test_safety_limit_stops(self):
        self.adapter.tracker.safety_speed_limit=0
        self.adapter.call(0,.05,self.inputs,self.outputs)
        self.assertEqual(self.outputs[0],0)
        self.assertEqual(self.outputs[1],0)


if __name__=='__main__':
    PARAMS=json.loads(Path(sys.argv.pop(1)).read_text())
    unittest.main()
