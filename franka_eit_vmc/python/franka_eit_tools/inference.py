"""Sensor-only inference and event hysteresis; never imports robot-control code."""
from pathlib import Path
import hashlib
import numpy as np
from .position_model import PositionModel, noise_level, signature

class Inference:
    def __init__(self, model_path, baseline=None, on_frames=3, off_frames=2):
        self.model=PositionModel.load(model_path)
        self.sha256=hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
        self.physical=self.model.metadata.get('demo') is False
        self.baseline=None
        self.on_frames=int(on_frames);self.off_frames=int(off_frames)
        if min(self.on_frames,self.off_frames)<1:
            raise ValueError('Event persistence must be positive')
        self.on=0;self.off=0;self.contact=False
        if baseline is not None:self.set_baseline(baseline)

    def set_baseline(self,capture):
        noise_level(capture)
        self.baseline=capture
        self.on=0;self.off=0;self.contact=False

    def step(self,frame):
        result={'contact':False,'measurement_valid':False,'position_valid':False,'x_m':0.,'y_m':0.,
                'signal_strength':0.,'numbering':self.model.metadata['numbering'],'model_is_physical':self.physical,'model_sha256':self.sha256}
        if self.baseline is None:
            return result
        capture={'frames':[frame]}
        try:
            _,strength=signature(capture,self.baseline)
            on=max(self.model.threshold,5*noise_level(self.baseline))
            if strength>on:
                self.on+=1;self.off=0
            elif strength<.7*on:
                self.off+=1;self.on=0
            else:
                self.on=0;self.off=0
            if self.on>=self.on_frames:self.contact=True
            if self.off>=self.off_frames:self.contact=False
            predicted=self.model.predict(capture,self.baseline)
            valid=self.contact and predicted['state']=='contact'
            xy=np.asarray(predicted['xy_mm'] if valid else [0.,0.],float)/1000.
            result.update(contact=self.contact,measurement_valid=True,position_valid=valid,x_m=float(xy[0]),y_m=float(xy[1]),signal_strength=float(strength))
        except (ValueError,FloatingPointError):
            # Bad/clipped electrical frames invalidate the contact stream immediately.
            self.on=0;self.off=0;self.contact=False
        return result
