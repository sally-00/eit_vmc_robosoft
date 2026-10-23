"""Engraving convention viewed from the contacting face, +Z pointing out of that face."""
import math
import numpy as np

def number_angle(number,numbering='ccw'):
    if not isinstance(number,int) or not 1<=number<=8 or numbering not in ('ccw','cw'):
        raise ValueError('Use engraved numbers 1–8 and numbering ccw/cw')
    return math.radians(-90+45*(number-1) if numbering=='ccw' else 90-45*(number-1))

def pair_yaw_for_base_x(pair,numbering='ccw'):
    if len(pair)!=2 or pair[0]==pair[1] or (pair[0]-pair[1])%8 not in (1,7):
        raise ValueError('Specify two DISTINCT ADJACENT engraved numbers, e.g. 4 5')
    a,b=(number_angle(n,numbering) for n in pair)
    direction=complex(math.cos(a)+math.cos(b),math.sin(a)+math.sin(b))
    return math.atan2(direction.imag,direction.real)

def ready_pad_rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,s,0.],[s,-c,0.],[0.,0.,-1.]])
