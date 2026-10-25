"""Sensor-only inference and event hysteresis; never imports robot-control code."""
from pathlib import Path
import hashlib
import math
import numpy as np
from .position_model import PositionModel

class Inference:
    def __init__(self, model_path, baseline=None, on_frames=3, off_frames=2,
                 contact_threshold=0.0):
        self.model=PositionModel.load(model_path)
        self.sha256=hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
        self.physical=self.model.metadata.get('demo') is False
        self.baseline=None
        self.on_frames=int(on_frames);self.off_frames=int(off_frames)
        if min(self.on_frames,self.off_frames)<1:
            raise ValueError('Event persistence must be positive')
        self.set_contact_threshold(contact_threshold)
        self.on=0;self.off=0;self.contact=False
        if baseline is not None:self.set_baseline(baseline)

    def set_contact_threshold(self,value):
        threshold=float(value)
        if not math.isfinite(threshold) or threshold < 0:
            raise ValueError('contact_threshold must be finite and nonnegative')
        self.contact_threshold=threshold

    def set_baseline(self,capture):
        self.model.noise_level(capture)
        self.baseline=capture
        self.on=0;self.off=0;self.contact=False

    def step(self,frame):
        result={'contact':False,'contact_candidate':False,'measurement_valid':False,'position_valid':False,'x_m':0.,'y_m':0.,
                'signal_strength':0.,'contact_threshold':self.contact_threshold,
                'pattern_distance':0.,'pattern_support_limit':self.model.support_limit,
                'predicted_radius_mm':0.,
                'numbering':self.model.metadata['numbering'],'model_is_physical':self.physical,'model_sha256':self.sha256}
        if self.baseline is None:
            return result
        capture={'frames':[frame]}
        try:
            predicted=self.model.predict(capture,self.baseline,self.contact_threshold)
            strength=predicted['strength']
            result.update(contact_candidate=predicted['state']=='contact',
                          pattern_distance=float(predicted['pattern_distance']),
                          predicted_radius_mm=float(np.linalg.norm(predicted['xy_mm'])))
            if predicted['state']=='contact':
                self.on+=1;self.off=0
            else:
                self.off+=1;self.on=0
            if self.on>=self.on_frames:self.contact=True
            if self.off>=self.off_frames:self.contact=False
            valid=self.contact and predicted['state']=='contact'
            xy=np.asarray(predicted['xy_mm'] if valid else [0.,0.],float)/1000.
            result.update(contact=self.contact,measurement_valid=True,position_valid=valid,x_m=float(xy[0]),y_m=float(xy[1]),signal_strength=float(strength))
        except (ValueError,FloatingPointError):
            # Bad/clipped electrical frames invalidate the contact stream immediately.
            self.on=0;self.off=0;self.contact=False
        return result
