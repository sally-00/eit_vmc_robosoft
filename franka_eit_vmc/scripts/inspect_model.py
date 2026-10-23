#!/usr/bin/env python3
"""Inspect a bundled/exported EIT model without ROS, a sensor or a robot."""
import argparse,json,hashlib
from pathlib import Path
from franka_eit_tools.position_model import PositionModel

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('model',type=Path);a=p.parse_args()
    model=PositionModel.load(a.model)
    print(json.dumps({'sha256':hashlib.sha256(a.model.read_bytes()).hexdigest(),'physical_model':model.metadata.get('demo') is False,'training_touches':len(model.x),'metadata':model.metadata},indent=2))
if __name__=='__main__':main()
