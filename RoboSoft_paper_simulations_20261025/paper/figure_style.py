"""Shared figure colors and contrast helpers; numerical data are unchanged."""
import numpy as np
from matplotlib.colors import to_rgb
INK = '#202124'
PHYSICAL = '#4B5563'
VIRTUAL = '#985000'
CONTACT = '#0072B2'
BASELINE_TEXT = '#993E00'
COLORS = {'centre':'#D55E00', 'odometry':'#D55E00', 'latched':'#CC79A7',
          'eit':'#0072B2', 'oracle':'#009E73',
          'timed_eit':'#666666', 'event_eit':'#0072B2'}
HATCHES = {'centre':'///', 'odometry':'///', 'latched':'...',
           'eit':'', 'oracle':'xx', 'timed_eit':'--', 'event_eit':''}

def luminance(rgb):
    values=np.asarray(rgb, dtype=float)[..., :3]
    linear=np.where(values <= .04045, values/12.92, ((values+.055)/1.055)**2.4)
    return linear @ np.array([.2126,.7152,.0722])

def contrast(a,b):
    la,lb=float(luminance(to_rgb(a))),float(luminance(to_rgb(b)))
    return (max(la,lb)+.05)/(min(la,lb)+.05)

def annotation_color(background):
    rgb=np.asarray(background)[:3]
    value=float(luminance(rgb))
    return '#000000' if (value+.05)/.05 >= 1.05/(value+.05) else '#FFFFFF'
