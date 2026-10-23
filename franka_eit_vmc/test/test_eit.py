#!/usr/bin/env python3
import json,math,sys,tempfile,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from franka_eit_tools.position_model import PositionModel,DemoReader,targets,fit_samples
from franka_eit_tools.inference import Inference
from franka_eit_tools.eit_reader import adjacent_protocol,parse_frame
from franka_eit_tools.mount import pair_yaw_for_base_x,ready_pad_rotation,number_angle

class Tests(unittest.TestCase):
    def test_frame_protocol(self):
        self.assertEqual(len(adjacent_protocol(list(range(1,9)))),40)
        line=','.join(['0.01|0']*39+['0.02|0.1C'])
        self.assertTrue(parse_frame(line)['clipped'][-1])
        with self.assertRaises(ValueError):parse_frame('nan|0,'*40)
        with self.assertRaises(ValueError):adjacent_protocol([1]*8)

    def test_demo_is_not_physical(self):
        path=ROOT/'models/demo_position_model.npz'
        engine=Inference(path)
        self.assertFalse(engine.physical)
        reader=DemoReader({'eit_frames_per_capture':5})
        baseline=reader.capture();engine.set_baseline(baseline)
        reader.xy=np.array([0.,0.]);frame=reader.capture()['frames'][0]
        for _ in range(3):out=engine.step(frame)
        self.assertTrue(out['contact']);self.assertFalse(out['model_is_physical'])
        bad=dict(frame,clipped=[True]*40)
        self.assertFalse(engine.step(bad)['position_valid'])

    def test_pair_mapping(self):
        for numbering in ['ccw','cw']:
            for first in range(1,9):
                second=first%8+1;angle=pair_yaw_for_base_x([first,second],numbering)
                alpha=number_angle(first,numbering);beta=number_angle(second,numbering)
                v=np.array([math.cos(alpha)+math.cos(beta),math.sin(alpha)+math.sin(beta),0.]);v/=np.linalg.norm(v)
                self.assertTrue(np.allclose(ready_pad_rotation(angle)@v,[1,0,0]))
        with self.assertRaises(ValueError):pair_yaw_for_base_x([5,5])

    def test_safe_npz_format(self):
        model=PositionModel.load(ROOT/'models/demo_position_model.npz')
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'model.npz';model.save(path);copy=PositionModel.load(path)
            self.assertTrue(np.allclose(copy.estimate(model.x)[0],model.estimate(model.x)[0]))

if __name__=='__main__':unittest.main()
