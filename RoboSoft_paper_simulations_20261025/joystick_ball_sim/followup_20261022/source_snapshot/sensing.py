"""Actual synthetic FEM voltages -> existing learned estimator, no force feedback."""
import sys
import numpy as np
from scene import ROOT, BASE, FACE_Z
sys.path.insert(0, str(BASE / 'src'))
from electrical import EIT, measure, session
import joblib

_MODEL = None
_FORWARD = None


class Sensor:
    def __init__(self, seed, cfg):
        global _MODEL, _FORWARD
        if _MODEL is None:
            _MODEL = joblib.load(ROOT/'results/eit_estimator_restored.joblib')
            _FORWARD = EIT(.0048, angle=.006)
        self.forward, self.estimator = _FORWARD, _MODEL
        self.pars = session(20000+seed, cfg.get('sensor_shift', False))
        self.baseline = self.forward.solve(**{k: self.pars[k] for k in ['sigma', 'z']})
        self.rng = np.random.default_rng(30000+seed)
        self.next = 0.; self.rate = cfg.get('rate', 20.)
        self.delay = cfg.get('delay', 0.)
        self.queue = []; self.signal_force = 0.
        self.location = np.array([0., 0., FACE_Z]); self.oracle = None
        self.valid = False; self.on = 0; self.off = 0; self.last = -np.inf
        self.force_estimate = 0.; self.norm = 0.; self.cfg = cfg

    def step(self, t, dt, force, local):
        self.signal_force += (force-self.signal_force)*min(1., dt/.08)
        acquired = False
        if t+1e-9 >= self.next:
            active = local is not None and np.linalg.norm(local[:2]) < .0445
            pars = self.pars.copy()
            if force < self.signal_force-.02:
                pars['gain'] *= 1.06
            feat = measure(self.forward, local[:2] if active else None,
                           self.signal_force if active else 0., pars, self.rng,
                           self.baseline)
            feat += self.rng.normal(0, self.cfg.get('noise', 0.), len(feat))
            pred = self.estimator.predict(feat[None])[0]*[.03, .03, 5.]
            self.queue.append((t+self.delay, t, np.linalg.norm(feat), pred,
                               local.copy() if active else None))
            self.next += 1/self.rate
        while self.queue and self.queue[0][0] <= t+1e-10:
            _, stamp, norm, pred, oracle = self.queue.pop(0)
            self.norm, self.last = float(norm), stamp
            if norm > .008:
                self.on += 1; self.off = 0
            elif norm < .005:
                self.off += 1; self.on = 0
            if self.on >= 3:
                self.valid = True
            if self.off >= 2:
                self.valid = False
            self.location = np.r_[pred[:2], FACE_Z]
            self.oracle = oracle
            self.force_estimate = float(pred[2])
            acquired = True
        if t-self.last > max(.15, 2/self.rate):
            self.valid = False
        return acquired
