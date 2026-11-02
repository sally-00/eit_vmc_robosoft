"""Archive integrity and missing-condition behavior; no hardware or live ROS domain."""
import base64
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from franka_eit_tools.recording import Archive,file_packet,initialize_recording,parameter_value,snapshot,process_identity

spec=importlib.util.spec_from_file_location('build_snapshot',ROOT/'scripts/generate_build_snapshot.py')
build_snapshot=importlib.util.module_from_spec(spec);spec.loader.exec_module(build_snapshot)


class RecordingTests(unittest.TestCase):
    def test_foreign_or_reused_pid_is_not_mistaken_for_local_controller(self):
        identity=process_identity()
        with tempfile.TemporaryDirectory() as folder:
            archive=Archive(folder)
            with self.assertRaisesRegex(RuntimeError,'remote or was replaced'):
                archive.process(identity['process_id'],'different-host',identity['process_start_ticks'])
            with self.assertRaisesRegex(RuntimeError,'remote or was replaced'):
                archive.process(identity['process_id'],identity['process_boot_id'],identity['process_start_ticks']+1)

    def test_preserves_original_model_after_source_file_changes(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);model=root/'model.npz';model.write_bytes(b'original loaded model')
            packet=file_packet(model,'loaded_eit_model')
            model.write_bytes(b'changed after loading')
            archive=Archive(root/'run')
            state=archive.unpack_packets(dict(artifacts=[packet],baseline={'frames':[1,2,3]}))
            artifact=state['archived_artifacts'][0]
            self.assertEqual((archive.root/artifact['archive_path']).read_bytes(),b'original loaded model')
            with self.assertRaisesRegex(RuntimeError,'SHA-256 mismatch'):
                archive.unpack_packets(dict(artifacts=[packet|{'data_base64':base64.b64encode(b'corrupt').decode()}]))

    def test_all_ros_parameter_types_are_preserved(self):
        fields={0:(None,None),1:('bool_value',False),2:('integer_value',123456789),
                3:('double_value',.00123),4:('string_value','two axes'),5:('byte_array_value',bytes([0,255])),
                6:('bool_array_value',[True,False]),7:('integer_array_value',[1,-3]),
                8:('double_array_value',[.1,.2]),9:('string_array_value',['x','y'])}
        for kind,(field,value) in fields.items():
            with self.subTest(kind=kind):
                message=SimpleNamespace(type=kind)
                if field:setattr(message,field,value)
                self.assertEqual(parameter_value(message),list(value) if kind>=5 else value)

    def test_configuration_and_physical_notes_are_content_addressed(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);config=root/'config';config.mkdir()
            (config/'ball.yaml').write_text('duration: 8\n')
            notes=root/'physical.json';notes.write_text('{"ball_radius_mm": 12.5}')
            path=initialize_recording(root/'run',config,['run_block','--task','ball'],notes)
            manifest=json.loads(Path(path).read_text())
            self.assertTrue(manifest['operator_conditions_provided'])
            entry=next(x for x in manifest['files'] if x['source_path'].endswith('ball.yaml'))
            (config/'ball.yaml').write_text('duration: 99\n')
            self.assertEqual((Path(path).parent/entry['archive_path']).read_text(),'duration: 8\n')

    def test_build_archive_contains_dirty_and_untracked_source(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);package=root/'franka_eit_vmc';(package/'src').mkdir(parents=True)
            tracked=package/'src/core.cpp';tracked.write_text('old mechanism\n')
            subprocess.run(['git','init','-q',str(root)],check=True)
            subprocess.run(['git','-C',str(root),'add','.'],check=True)
            subprocess.run(['git','-C',str(root),'-c','user.name=Snapshot Test','-c',
                            'user.email=snapshot@example.invalid','commit','-qm','initial'],check=True)
            tracked.write_text('new mechanism\n')
            (package/'src/new.cpp').write_text('untracked control source\n')
            output=root/'build';output.mkdir();(output/'CMakeCache.txt').write_text('FLAGS=release\n')
            manifest=build_snapshot.generate(package,output/'provenance',output)
            data=base64.b64decode(manifest['source_archive']['data_base64'])
            with tarfile.open(fileobj=io.BytesIO(data),mode='r:gz') as archive:
                self.assertEqual(archive.extractfile('franka_eit_vmc/src/core.cpp').read(),b'new mechanism\n')
                self.assertEqual(archive.extractfile('franka_eit_vmc/src/new.cpp').read(),b'untracked control source\n')
                self.assertIn(b'new mechanism',archive.extractfile('build/git_diff.patch').read())
            self.assertEqual(hashlib.sha256(data).hexdigest(),manifest['source_archive']['sha256'])
            self.assertIn('recording_build_manifest',(output/'provenance/recording_build.hpp').read_text())

    def test_missing_node_is_recorded_and_prevents_motion_for_both_tasks(self):
        for task in ('joystick','ball'):
            with self.subTest(task=task),tempfile.TemporaryDirectory() as folder:
                node=SimpleNamespace(get_node_names_and_namespaces=lambda:[])
                with patch('franka_eit_tools.recording.all_parameters',side_effect=RuntimeError('missing sensor')):
                    with self.assertRaisesRegex(RuntimeError,'not fully recorded'):
                        snapshot(node,folder,'attempt.before_motion',task,dict(condition='baseline'))
                path=Path(folder)/'reproducibility/snapshots/attempt.before_motion.json'
                saved=json.loads(path.read_text())
                self.assertEqual(saved['status'],'incomplete')
                self.assertIn('missing sensor',saved['error'])


if __name__=='__main__':unittest.main()
