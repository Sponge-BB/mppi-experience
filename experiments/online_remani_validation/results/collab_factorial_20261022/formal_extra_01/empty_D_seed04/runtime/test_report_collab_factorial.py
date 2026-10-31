import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from report_collab_factorial import read,lines,unpacked,paired_measure,static_reference_projection


class ReportContracts(unittest.TestCase):
    def test_compressed_and_plain_read_equal(self):
        with tempfile.TemporaryDirectory(prefix='collab_report_') as directory:
            p=Path(directory)/'data.json';value={'a':[1,2,3]}
            with gzip.open(str(p)+'.gz','wt') as f:json.dump(value,f)
            self.assertEqual(read(p),value)
            q=p.with_name('rows.jsonl')
            with gzip.open(str(q)+'.gz','wt') as f:f.write('{"a":1}\n{"a":2}\n')
            self.assertEqual(lines(q),[{'a':1},{'a':2}])

    def test_unpack_is_verified_and_recoverable(self):
        with tempfile.TemporaryDirectory(prefix='collab_report_') as directory:
            root=Path(directory);blob=b'{"a":1}\n';archive=root/'data.jsonl.gz'
            with gzip.open(archive,'wb') as f:f.write(blob)
            record={'data.jsonl':dict(original_sha256=hashlib.sha256(blob).hexdigest(),
                    gzip_sha256=hashlib.sha256(archive.read_bytes()).hexdigest())}
            (root/'compression_audit.json').write_text(json.dumps(record))
            with unpacked(root):self.assertEqual((root/'data.jsonl').read_bytes(),blob)
            self.assertFalse((root/'data.jsonl').exists());self.assertTrue(archive.exists())

    def test_missing_value_is_not_zero(self):
        self.assertIsNone(read(Path('/tmp/nonexistent_collab_record_20261022.json')))

    def test_paired_targets_must_match_within_goal_policy(self):
        with tempfile.TemporaryDirectory(prefix='collab_report_') as directory:
            root=Path(directory)
            (root/'paired_summary.json').write_text('{"paired_integrity":true}')
            (root/'manifest.json').write_text('{"scene":"static","factorial":{"seed":1}}')
            records=[]
            for i,g in enumerate('ABCDDCBA'):
                records.append(dict(event='collab_pair_result',group=g,label=str(i),task_hash='task',
                    target_hash='wrong' if g=='B' else 'goal',stable=True,target_ok=True,valid=True,wall_ms=1))
            (root/'dual_timeline.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in records))
            _,audit=paired_measure(root)
            self.assertFalse(audit['valid']);self.assertTrue(any('target mismatch' in x for x in audit['errors']))

    def test_eligible_population_is_not_pooled_with_first_refresh(self):
        with tempfile.TemporaryDirectory(prefix='collab_report_') as directory:
            parent=Path(directory);root=parent/'trial';root.mkdir()
            (parent/'experiment_plan.json').write_text('{"pair_eligible":true}')
            (root/'paired_summary.json').write_text('{"paired_integrity":true}')
            (root/'manifest.json').write_text('{"scene":"static","factorial":{"seed":2,"group":"B"}}')
            records=[dict(event='collab_pair_result',group=g,label=str(i),task_hash='task',
                target_hash='goal',stable=True,target_ok=True,valid=False,wall_ms=1)
                for i,g in enumerate('ABCDDCBA')]
            (root/'dual_timeline.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in records))
            rows,audit=paired_measure(root)
            self.assertTrue(audit['valid'])  # failed solves are valid experiment outcomes
            self.assertTrue(all(r['population']=='first_eligible_prefix' and r['donor_group']=='B' for r in rows))

    def test_reference_projection_uses_body_and_execution_margin(self):
        with tempfile.TemporaryDirectory(prefix='collab_report_') as directory:
            root=Path(directory)
            (root/'geometry.csv').write_text('0,0,0.2\n')
            (root/'state.csv').write_text('t,x,y,yaw\n0,0.4,0,0\n2,0.4,0,0\n')
            (root/'obstacles.csv').write_text('t,x,y,r,id,vx,vy\n0,1,0,0.2,1,0,0\n')
            (root/'limits.json').write_text('{"margin":0.15}')
            reference=[[.5,0,0,.2],[.5,1,0,.2]]
            (root/'consumed_references.jsonl').write_text(''.join(json.dumps(dict(t=1,revision=i,reference=reference))+'\n' for i in [1,2]))
            rows=static_reference_projection([dict(path=str(root),scene='static',group='C',seed=2)])
            self.assertEqual(len(rows),1)  # exclude original reference before unknown obstacle
            self.assertAlmostEqual(rows[0]['minimum_sampled_projection_gap_m'],.1)
            self.assertTrue(rows[0]['below_execution_margin'])
            self.assertAlmostEqual(rows[0]['reference_start_to_actual_position_m'],.1)


if __name__=='__main__':unittest.main()
