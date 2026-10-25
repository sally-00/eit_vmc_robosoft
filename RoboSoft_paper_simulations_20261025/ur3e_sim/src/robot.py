"""Official UR3e link transforms and inertias; simplified collision/visual geometry."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml, mujoco
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
NAMES=['shoulder','upper_arm','forearm','wrist_1','wrist_2','wrist_3']
LIMITS=np.array([54,54,28,9,9,9.])
def fmt(a):return ' '.join(f'{v:.12g}' for v in np.asarray(a).ravel())
def quat(r):
    q=Rotation.from_euler('xyz',r).as_quat();return fmt(q[[3,0,1,2]])

def make_model(spring=0.,mu=.6,soft=.008,dt=.001):
    cfg=ROOT/'vendor/ur_description/config/ur3e'
    kin=yaml.safe_load((cfg/'default_kinematics.yaml').read_text())['kinematics']
    phys=yaml.safe_load((cfg/'physical_parameters.yaml').read_text())['inertia_parameters']
    root=ET.Element('mujoco',model='UR3e eight-electrode EIT POC')
    ET.SubElement(root,'compiler',angle='radian',autolimits='true')
    ET.SubElement(root,'option',timestep=str(dt),integrator='implicitfast',gravity='0 0 -9.81',cone='elliptic',iterations='80',tolerance='1e-10')
    vis=ET.SubElement(root,'visual');ET.SubElement(vis,'global',offwidth='1200',offheight='900')
    default=ET.SubElement(root,'default');ET.SubElement(default,'geom',contype='0',conaffinity='0')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='0 -1 2',dir='0 0 -1',diffuse='.8 .8 .8')
    ET.SubElement(world,'geom',type='plane',size='1 1 .01',rgba='.93 .94 .96 1')
    parent=ET.SubElement(world,'body',name='base',quat=quat([0,0,np.pi]))
    ET.SubElement(parent,'geom',type='cylinder',size='.065 .025',pos='0 0 .025',rgba='.3 .35 .4 1')
    for i,name in enumerate(NAMES):
        k=kin[name];p=[k[a] for a in ['x','y','z']];r=[k[a] for a in ['roll','pitch','yaw']]
        body=ET.SubElement(parent,'body',name=name,pos=fmt(p),quat=quat(r))
        ET.SubElement(body,'joint',name=f'j{i}',axis='0 0 1',range=('-3.14159 3.14159' if i==2 else '-6.28318 6.28318'),damping='.15',armature='.015')
        cog=phys['center_of_mass'][name+'_cog'];rot=phys['rotation'][name];it=phys['tensor'][name]
        I=np.array([[it['ixx'],it['ixy'],it['ixz']],[it['ixy'],it['iyy'],it['iyz']],[it['ixz'],it['iyz'],it['izz']]])
        R=Rotation.from_euler('xyz',[rot[k] for k in ['roll','pitch','yaw']]).as_matrix();I=R@I@R.T
        ET.SubElement(body,'inertial',mass=str(phys[name+'_mass']),pos=fmt([cog[k] for k in ['x','y','z']]),fullinertia=fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
        ET.SubElement(body,'geom',type='cylinder',size='.033 .032',rgba='.45 .68 .8 1')
        if i<5:
            n=kin[NAMES[i+1]];end=np.array([n[a] for a in ['x','y','z']])
            if np.linalg.norm(end)>.05:ET.SubElement(body,'geom',type='capsule',size='.025',fromto=fmt(np.r_[np.zeros(3),end]),rgba='.7 .73 .76 1')
        parent=body
    # UR macro's wrist_3 -> flange -> tool0 rotations, combined explicitly.
    Rf=Rotation.from_euler('xyz',[0,-np.pi/2,-np.pi/2]).as_matrix()
    Rt=Rotation.from_euler('xyz',[np.pi/2,0,np.pi/2]).as_matrix()
    q=Rotation.from_matrix(Rf@Rt).as_quat()
    tool=ET.SubElement(parent,'body',name='tool',quat=fmt(q[[3,0,1,2]]))
    ET.SubElement(tool,'inertial',mass='.35',pos='0 0 .01675',diaginertia='.000402 .000402 .000739')
    ET.SubElement(tool,'geom',name='housing',type='cylinder',size='.065 .01525',pos='0 0 .01525',rgba='.18 .23 .3 1',contype='1',conaffinity='2')
    ET.SubElement(tool,'geom',name='front',type='cylinder',size='.065 .0015',pos='0 0 .032',rgba='.5 .52 .55 1',contype='1',conaffinity='2',friction=f'{mu} .001 .0001',solref=f'{soft} 1',solimp='.95 .99 .001',condim='3')
    ET.SubElement(tool,'geom',type='cylinder',size='.045 .00015',pos='0 0 .0335',rgba='.1 .64 .58 .95')
    for j in range(8):
        a=j*np.pi/4;ET.SubElement(tool,'geom',type='sphere',size='.002',pos=fmt([.045*np.cos(a),.045*np.sin(a),.0335]),rgba='.95 .68 .18 1')
    ET.SubElement(tool,'site',name='sensor',pos='0 0 .0335',size='.002')
    support=ET.SubElement(world,'body',name='support',pos='.3 0 .2')
    if spring:
        ET.SubElement(support,'joint',name='support_z',type='slide',axis='0 0 1',stiffness=str(spring),damping='8',springref='0',range='-.04 .02')
        ET.SubElement(support,'inertial',mass='.05',pos='0 0 0',diaginertia='.00001 .00001 .00001')
    ET.SubElement(support,'geom',name='tip',type='sphere',size='.008',rgba='.9 .46 .19 1',contype='2',conaffinity='1',friction=f'{mu} .001 .0001',solref=f'{soft} 1',solimp='.95 .99 .001',condim='3')
    ET.SubElement(support,'geom',type='cylinder',size='.006 .04',pos='0 0 -.04',rgba='.4 .43 .5 1')
    act=ET.SubElement(root,'actuator')
    for i,lim in enumerate(LIMITS):ET.SubElement(act,'motor',name=f'm{i}',joint=f'j{i}',ctrlrange=f'{-lim} {lim}')
    return mujoco.MjModel.from_xml_string(ET.tostring(root).decode()),ET.tostring(root).decode()

def ik(model,target,Rtarget):
    d=mujoco.MjData(model);d.qpos[:6]=[0,-1.2,1.7,-2.,-1.5,0]
    sid=model.site('sensor').id
    jp=np.zeros((3,model.nv));jr=np.zeros_like(jp)
    for it in range(1200):
        mujoco.mj_forward(model,d)
        err=np.r_[target-d.site_xpos[sid],Rotation.from_matrix(Rtarget@d.site_xmat[sid].reshape(3,3).T).as_rotvec()]
        if np.linalg.norm(err)<1e-7:return d.qpos.copy()
        mujoco.mj_jacSite(model,d,jp,jr,sid);J=np.vstack([jp[:,:6],jr[:,:6]])
        dq=J.T@np.linalg.solve(J@J.T+1e-4*np.eye(6),err)
        d.qpos[:6]+=np.clip(dq,-.08,.08)
    raise RuntimeError(f'IK residual {np.linalg.norm(err)}')

if __name__=='__main__':
    m,xml=make_model();q=ik(m,np.array([.3,0,.25]),np.diag([1,-1,-1]));print(q)
    (ROOT/'results'/'ur3e_scene.xml').write_text(xml)
