#!/usr/bin/env python3
"""Collect manually labelled physical touches, without connecting to a robot."""
import argparse,json
from datetime import datetime,timezone
from pathlib import Path
from franka_eit_tools.eit_reader import EITReader
from franka_eit_tools.position_model import targets,new_session

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--points',type=int,default=20);p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--numbering',choices=['ccw','cw'],default='ccw')
    args=p.parse_args();plan=targets(args.points)
    if args.repeats<2:p.error('Use at least two independent touches per location')
    cfg=json.loads(args.config.read_text());reader=EITReader(cfg)
    try:
        reader.open()
        metadata={'created_utc':datetime.now(timezone.utc).isoformat(),'demo':False,'numbering':args.numbering,'radius_mm':45.,'margin_mm':5.,'targets_mm':plan.tolist(),'eit_config':cfg,'device':reader.metadata(),'label':'contact position only, millimetres'}
        session=new_session(args.output,metadata);print('Session:',session)
        print('Coordinates are viewed from contacting face: CCW 1 bottom, 3 right, 5 top, 7 left. Mark known positions independently.')
        with (session/'touches.jsonl').open('w') as stream:
            for repeat in range(args.repeats):
                for point_id,xy in enumerate(plan):
                    input(f'Round {repeat+1}, point {point_id+1}: release pad; Enter to capture baseline. ')
                    baseline=reader.capture()
                    input(f'Touch known pad X={xy[0]:.1f}, Y={xy[1]:.1f} mm; hold, then Enter. ')
                    record={'point_id':point_id,'repeat':repeat,'xy_mm':xy.tolist(),'baseline':baseline,'contact':reader.capture()}
                    stream.write(json.dumps(record)+'\n');stream.flush()
        print('Train with: ros2 run franka_eit_vmc train_eit.py --session',session,'--output /path/to/position_model.npz')
    finally:reader.close()
if __name__=='__main__':main()
