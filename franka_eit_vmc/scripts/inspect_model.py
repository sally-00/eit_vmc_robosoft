#!/usr/bin/env python3
"""Inspect the newest trained EIT model without ROS, a sensor or a robot."""
import argparse,json,hashlib
from pathlib import Path
from franka_eit_tools.position_model import PositionModel
from franka_eit_tools.model_selection import select_model

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('model',type=Path,nargs='?');a=p.parse_args()
    selected=select_model(a.model)
    model=PositionModel.load(selected)
    print(json.dumps({'model_path':str(selected),'sha256':hashlib.sha256(selected.read_bytes()).hexdigest(),'physical_model':model.metadata.get('demo') is False,'support_vectors':len(model.x),'metadata':model.metadata},indent=2))
if __name__=='__main__':main()
