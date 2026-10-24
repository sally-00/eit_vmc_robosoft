#!/usr/bin/env python3
import json,math,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from franka_eit_tools.position_model import PositionModel
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
        base={'magnitude_Vrms':[.01]*40,'phase_rad':[0.]*40,'clipped':[False]*40}
        engine.set_baseline({'frames':[base]*5})
        x=engine.model.x[0];strength=max(.01,2*engine.model.threshold)
        voltage=.01*(1+strength*(x[:40]+1j*x[40:]))
        frame={'magnitude_Vrms':np.abs(voltage).tolist(),'phase_rad':np.angle(voltage).tolist(),'clipped':[False]*40}
        for _ in range(3):out=engine.step(frame)
        self.assertTrue(out['contact']);self.assertFalse(out['model_is_physical'])
        self.assertTrue(out['position_valid'])
        np.testing.assert_allclose([out['x_m'],out['y_m']],engine.model.estimate(x)[0][0]/1000,atol=1e-10)
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

    def test_read_only_loading(self):
        path=ROOT/'models/demo_position_model.npz'
        with patch('numpy.linalg.solve',side_effect=AssertionError('Runtime must not fit')):
            model=PositionModel.load(path)
            with np.load(path,allow_pickle=False) as data:
                d=model.distances(model.x,model.x)
                expected=np.exp(-d**2/(2*float(data['width'])**2))@data['weights']
            np.testing.assert_allclose(model.estimate(model.x)[0],expected)
        self.assertFalse(hasattr(model,'save'))

    def test_reject_unexported_and_invalid_models(self):
        with np.load(ROOT/'models/demo_position_model.npz',allow_pickle=False) as data:
            arrays={key:data[key] for key in data.files}
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'model.npz'
            legacy={k:v for k,v in arrays.items() if k not in ('weights','width','support_limit')}
            np.savez(path,**legacy)
            with self.assertRaisesRegex(ValueError,'export_franka_model.py'):PositionModel.load(path)
            for field,value in [('weights',np.full_like(arrays['weights'],np.nan)),('width',0.),('x',np.ones((3,20)))]:
                np.savez(path,**{**arrays,field:value})
                with self.assertRaises(ValueError):PositionModel.load(path)
            np.savez(path,**{**arrays,'weights':np.array([object()],dtype=object)})
            with self.assertRaises(ValueError):PositionModel.load(path)

if __name__=='__main__':unittest.main()
