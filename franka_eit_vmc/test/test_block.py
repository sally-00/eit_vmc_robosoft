"""Operator pacing and condition ordering, with every hardware call mocked."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('run_block',Path(__file__).resolve().parents[1]/'scripts/run_block.py')
block=importlib.util.module_from_spec(spec);spec.loader.exec_module(block)


class BlockTests(unittest.TestCase):
    def test_two_conditions_are_paced_and_logged(self):
        with tempfile.TemporaryDirectory() as d:
            args=['run_block.py','--task','joystick','--block','center','--repeats','2','--output',d]
            with patch.object(sys,'argv',args),patch.object(block,'check_controller',return_value={'landing_offset_x_m':0.}),patch.object(block.subprocess,'run') as run,patch('builtins.input',side_effect=['ready','run','video1','ready','run','video2','ready','run','video3','ready','run','video4']):
                block.main()
            self.assertEqual(run.call_count,8)
            trials=[c.args[0] for c in run.call_args_list if 'trial' in c.args[0]]
            self.assertEqual([x[x.index('--condition')+1] for x in trials],['baseline','eit','eit','baseline'])
            outcomes=list(Path(d).rglob('outcomes.jsonl'))
            records=[json.loads(line) for line in outcomes[0].read_text().splitlines()]
            self.assertEqual([r['repeat'] for r in records],[1,1,2,2])

    def test_quit_never_starts_motion(self):
        with tempfile.TemporaryDirectory() as d:
            with patch.object(sys,'argv',['run_block.py','--task','ball','--block','push','--output',d]),patch.object(block,'check_controller',return_value={}),patch.object(block.subprocess,'run') as run,patch('builtins.input',return_value='q'):
                block.main()
            run.assert_not_called()

    def test_failed_operation_stops_block(self):
        with tempfile.TemporaryDirectory() as d:
            with patch.object(sys,'argv',['run_block.py','--task','ball','--block','undisturbed','--output',d]),patch.object(block,'check_controller',return_value={}),patch.object(block.subprocess,'run',side_effect=subprocess.CalledProcessError(1,'experiment')) as run,patch('builtins.input',return_value='ready'):
                with self.assertRaises(subprocess.CalledProcessError):block.main()
            self.assertEqual(run.call_count,1)

    def test_configuration_failure_prevents_motion(self):
        with patch.object(sys,'argv',['run_block.py','--task','joystick','--block','x_plus']),patch.object(block,'check_controller',side_effect=RuntimeError('wrong offset')),patch.object(block.subprocess,'run') as run:
            with self.assertRaisesRegex(RuntimeError,'wrong offset'):block.main()
        run.assert_not_called()

if __name__=='__main__':unittest.main()
