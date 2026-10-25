"""UR3e task scenes; reuse pinned robot inertias, add physical task objects."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT.parent / 'ur3e_sim'
sys.path.insert(0, str(BASE / 'src'))
from robot import make_model, ik, LIMITS, fmt

R0 = np.diag([1., -1., -1.])
FACE_Z = .0335


def make_scene(cfg):
    _, xml = make_model(dt=cfg.get('dt', .001), mu=cfg.get('mu', .7))
    root = ET.fromstring(xml)
    world = root.find('worldbody')
    world.remove(world.find("body[@name='support']"))
    world.find("geom[@type='plane']").set('rgba', '.94 .95 .97 1')
    root.find('option').set('iterations', '60')
    tool = root.find(".//body[@name='tool']")
    front = tool.find("geom[@name='front']")
    front.set('size', '.045 .0015')
    front.set('rgba', '.15 .64 .58 1')
    # The backing stops behind the pad. Segments model an assumed recessed rim.
    housing = tool.find("geom[@name='housing']")
    housing.set('contype', '0'); housing.set('conaffinity', '0')
    recess = cfg.get('recess', .003)
    for j in range(32):
        theta = 2*np.pi*j/32
        ET.SubElement(tool, 'geom', name=f'rim_{j}', type='box',
                      size=fmt([.009, .00555, .005]),
                      pos=fmt([.056*np.cos(theta), .056*np.sin(theta), FACE_Z+recess-.005]),
                      euler=fmt([0, 0, theta]), rgba='.3 .34 .4 1',
                      contype='1', conaffinity='2', friction='.7 .001 .0001',
                      solref='.008 1', solimp='.95 .99 .001', condim='3')
    # Restore the missing visual base shaft, without adding a collision surface.
    ET.SubElement(root.find(".//body[@name='base']"), 'geom', type='capsule',
                  size='.034', fromto='0 0 .025 0 0 .15185',
                  rgba='.7 .73 .76 1', contype='0', conaffinity='0')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', type='skybox', builtin='flat',
                  rgb1='.96 .97 .98', rgb2='.96 .97 .98', width='128', height='128')
    ET.SubElement(root.find('visual'), 'headlight', ambient='.6 .6 .6',
                  diffuse='.7 .7 .7', specular='.2 .2 .2')
    mu = cfg.get('mu', .7)
    gap = cfg.get('gap', .006)
    top = .29-gap
    offset = cfg.get('offset', 0.)
    task = cfg['task']
    if task == 'joystick':
        length, cap = cfg.get('lever_length', .08), cfg.get('cap_radius', .012)
        pivot = np.array([.3+offset, 0., top-length-cap])
        body = ET.SubElement(world, 'body', name='object', pos=fmt(pivot))
        ET.SubElement(body, 'joint', name='stick_angle', type='hinge', axis='0 1 0',
                      stiffness=str(cfg.get('stick_k', .15)), damping='.025',
                      range='-.35 .35', armature='.0001')
        ET.SubElement(body, 'inertial', mass='.065', pos=fmt([0, 0, .055]),
                      diaginertia='.00025 .00025 .00002')
        ET.SubElement(body, 'geom', name='stem', type='capsule', size='.003',
                      fromto=fmt([0, 0, 0, 0, 0, length]), rgba='.35 .38 .43 1')
        ET.SubElement(body, 'geom', name='cap', type='sphere', size=str(cap),
                      pos=fmt([0, 0, length]), rgba='.92 .51 .23 1',
                      contype='2', conaffinity='1', friction=f'{mu} .001 .0001',
                      solref='.008 1', solimp='.95 .99 .001', condim='3')
        ET.SubElement(world, 'geom', name='joystick_base', type='box',
                      size='.05 .04 .015', pos=fmt(pivot-[0, 0, .018]), rgba='.3 .34 .4 1')
        radius = cap
        initial_object = np.array([.3+offset, 0, top-cap])
    else:
        radius = cfg.get('radius', .025)
        table_z = top-2*radius
        table = ET.SubElement(world, 'body', name='table', mocap='true', pos=fmt([.4, 0, table_z-.02]))
        ET.SubElement(table, 'geom', name='tabletop', type='box', size='.35 .22 .02',
                      rgba='.78 .8 .82 1', contype='4', conaffinity='2',
                      friction=f"{cfg.get('table_mu', mu)} .0002 .0001",
                      solref='.008 1', solimp='.95 .99 .001', condim='3')
        initial_object = np.array([.3+offset, 0, table_z+radius])
        body = ET.SubElement(world, 'body', name='object', pos=fmt(initial_object))
        ET.SubElement(body, 'freejoint', name='ball_free')
        mass = cfg.get('mass', 1100*4/3*np.pi*radius**3)
        ET.SubElement(body, 'geom', name='ball', type='sphere', size=str(radius),
                      mass=str(mass), rgba='.9 .48 .2 1', contype='2', conaffinity='5',
                      friction=f'{mu} .0002 .0001', solref='.008 1',
                      solimp='.95 .99 .001', condim='3')
        # Non-colliding colored markers make actual sphere rotation visible.
        for j, axis in enumerate(np.eye(3)):
            ET.SubElement(body, 'geom', type='sphere', size='.002',
                          pos=fmt(axis*(radius+.0001)), rgba=['.15 .2 .3 1', '.95 .95 .95 1', '.2 .7 .8 1'][j],
                          contype='0', conaffinity='0', mass='0')
        pivot = None
    xml = ET.tostring(root).decode()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    q0 = ik(model, np.array([.3, 0, .29]), R0)
    data.qpos[:6] = q0[:6]
    if task == 'ball':
        # ik() starts object coordinates at defaults; retain the sphere quaternion.
        data.qpos[6:9] = initial_object
        data.qpos[9:13] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    info = dict(pivot=pivot, top=top, initial_object=initial_object, radius=radius,
                contact_names=['cap'] if task == 'joystick' else ['ball'])
    return model, data, xml, info


def observe_contact(model, data):
    """Evaluator truth used only to synthesize voltages, oracle, and common guards."""
    body = model.body('tool').id
    origin = data.xpos[body]
    rot = data.xmat[body].reshape(3, 3)
    pad = model.geom('front').id
    obj = model.body('object').id
    force = 0.; point = np.zeros(3); rim_force = 0.; slip = 0.
    contact_force = np.zeros(6)
    for i in range(data.ncon):
        contact = data.contact[i]
        gids = [contact.geom1, contact.geom2]
        if not any(model.geom_bodyid[g] == obj for g in gids):
            continue
        other = next((g for g in gids if model.geom_bodyid[g] == body), None)
        if other is None:
            continue
        mujoco.mj_contactForce(model, data, i, contact_force)
        normal = max(0., contact_force[0])
        if other != pad:
            rim_force += normal
            continue
        force += normal; point += normal*contact.pos
        j1 = np.zeros((3, model.nv)); j2 = np.zeros_like(j1)
        mujoco.mj_jac(model, data, j1, None, contact.pos, body)
        mujoco.mj_jac(model, data, j2, None, contact.pos, obj)
        relative = (j1-j2)@data.qvel
        n = contact.frame.reshape(3, 3)[0]
        slip += normal*np.linalg.norm(relative-n*np.dot(relative, n))
    if force > 1e-6:
        point /= force
        local = rot.T@(point-origin)
        local[2] = FACE_Z
        return force, local, point, rim_force, slip/force
    return 0., None, None, rim_force, 0.
