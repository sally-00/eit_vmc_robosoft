#!/usr/bin/env python3
"""Compute pad yaw from two adjacent engraved numbers; outputs a ROS parameter overlay."""
import argparse,math
from pathlib import Path
import yaml
from franka_eit_tools.mount import pair_yaw_for_base_x

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pair',type=int,nargs=2,required=True);p.add_argument('--numbering',choices=['ccw','cw'],default='ccw')
    p.add_argument('--view',choices=['contacting-face'],required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();yaw=pair_yaw_for_base_x(a.pair,a.numbering)
    params={'ready_pad_yaw_rad':yaw,'mount_orientation_confirmed':True,'pad_numbering':a.numbering}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text('# Pair '+str(a.pair)+' viewed from contacting face; midpoint points Base +X.\n'+yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=False))
    print('Ready pad yaw:',math.degrees(yaw),'degrees; saved',a.output)
    print('This does not measure EE-to-pad extrinsics; configure those separately.')
if __name__=='__main__':main()
