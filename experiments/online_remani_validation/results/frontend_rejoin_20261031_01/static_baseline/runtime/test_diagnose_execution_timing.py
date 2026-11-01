"""Check that tracing adds observations only, leaving the loop AST intact."""
import ast
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from diagnose_execution_timing import instrument_source, LoopProbe

HERE = Path(__file__).resolve().parent
FROZEN = HERE / 'results/execution_timing_pair_20261028_01/dynamic_head_on_center_matched_seed11/runtime/local_execution.py'


class RemoveProbe(ast.NodeTransformer):
    def visit_ImportFrom(self, node):
        if node.module in ('diagnose_execution_timing', 'rosgraph_msgs.msg'):
            return None
        return node

    def visit_Assign(self, node):
        target = node.targets[0]
        if isinstance(target, ast.Attribute) and target.attr in ('timing_probe', 'timing_clock_sub'):
            return None
        return self.generic_visit(node)

    def visit_Expr(self, node):
        call = node.value
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute):
            parent = call.func.value
            if isinstance(parent, ast.Attribute) and parent.attr == 'timing_probe':
                return None
        return self.generic_visit(node)


class TimingDiagnosisTests(unittest.TestCase):
    def test_instrumentation_does_not_change_original_logic(self):
        source = FROZEN.read_text()
        instrumented = ast.parse(instrument_source(source))
        stripped = RemoveProbe().visit(instrumented)
        self.assertEqual(ast.dump(stripped), ast.dump(ast.parse(source)))

    def test_in_memory_stages_and_clock_receipt_written_at_close(self):
        with tempfile.TemporaryDirectory(prefix='mppi_lag_probe_') as folder:
            probe = LoopProbe(folder, lambda: 1000.)
            probe.begin(1)
            probe.mark('command_published')
            probe.clock(SimpleNamespace(clock=SimpleNamespace(to_sec=lambda: 1000.)))
            self.assertFalse((Path(folder) / 'loop_stages.json').exists())
            probe.close()
            data = json.loads((Path(folder) / 'loop_stages.json').read_text())
            self.assertEqual([r[1] for r in data['rows']], ['cycle_start', 'command_published'])
            self.assertEqual(data['clocks'][0][1], 1000.)
            self.assertGreaterEqual(data['rows'][1][2], data['rows'][0][2])


if __name__ == '__main__':
    unittest.main()
