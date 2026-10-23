#!/usr/bin/env python3
"""Generate a labelled synthetic wiring/test model; NEVER a physical-pad calibration."""
import argparse,json
from pathlib import Path
from franka_eit_tools.position_model import DemoReader,targets,fit_samples

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    cfg={'eit_electrode_pins':list(range(1,9)),'eit_frequency_khz':50,'eit_periods_per_measurement':1,'eit_frames_per_capture':5}
    reader=DemoReader(cfg);records=[]
    for repeat in range(3):
        for i,xy in enumerate(targets(20)):
            reader.xy=None;baseline=reader.capture();reader.xy=xy
            records.append({'point_id':i,'repeat':repeat,'xy_mm':xy.tolist(),'baseline':baseline,'contact':reader.capture()})
    meta={'demo':True,'numbering':'ccw','radius_mm':45.,'eit_config':cfg,'description':'Synthetic UI-signal model for offline tests only; not a measured EIT pad'}
    model,report=fit_samples(records,meta);a.output.parent.mkdir(parents=True,exist_ok=True);model.save(a.output)
    print('DEMO ONLY:',a.output);print(json.dumps(report,indent=2))
if __name__=='__main__':main()
