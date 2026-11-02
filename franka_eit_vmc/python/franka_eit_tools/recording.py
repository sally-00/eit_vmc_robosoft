"""Content-addressed experiment provenance, collected outside robot motion."""
import base64
import hashlib
import importlib.metadata
import json
import os
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml


SCHEMA_VERSION = 1


def process_identity():
    fields=Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()
    return dict(process_id=os.getpid(),process_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                process_start_ticks=int(fields[19]))


def json_default(value):
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, 'tolist'):
        return value.tolist()
    if hasattr(value, 'get_fields_and_field_types'):
        return {name:getattr(value,name) for name in value.get_fields_and_field_types()}
    raise TypeError(type(value).__name__)


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data, indent=2, default=json_default, allow_nan=False)+'\n')
    temporary.replace(path)


def record_command(folder, role, command, **result):
    root=Path(folder)/'reproducibility'
    root.mkdir(parents=True,exist_ok=True)
    with (root/'commands.jsonl').open('a') as stream:
        stream.write(json.dumps(dict(time_utc=datetime.now(timezone.utc).isoformat(),
            role=role,argv=command,**result),default=json_default,allow_nan=False)+'\n')


def record_parameter_event(folder,event):
    root=Path(folder)/'reproducibility'
    root.mkdir(parents=True,exist_ok=True)
    with (root/'parameter_events.jsonl').open('a') as stream:
        stream.write(json.dumps(dict(received_host_unix_ns=time.time_ns(),event=event),
                                default=json_default,allow_nan=False)+'\n')


def file_packet(path, role):
    """Cache bytes at node startup, so later file edits cannot change the record."""
    path = Path(path)
    data = path.read_bytes()
    return dict(source_path=str(path.resolve()), role=role,
                sha256=hashlib.sha256(data).hexdigest(), data_base64=base64.b64encode(data).decode())


def python_packets(script):
    paths = {Path(script).resolve()}
    paths.update(Path(m.__file__).resolve() for name, m in list(sys.modules.items())
                 if name.startswith('franka_eit_tools.') and getattr(m, '__file__', None)
                 and Path(m.__file__).suffix == '.py')
    return [file_packet(p, 'loaded_python_source') for p in sorted(paths)]


def parameter_value(value):
    fields = {0: None, 1: 'bool_value', 2: 'integer_value', 3: 'double_value',
              4: 'string_value', 5: 'byte_array_value', 6: 'bool_array_value',
              7: 'integer_array_value', 8: 'double_array_value', 9: 'string_array_value'}
    if value.type not in fields:
        raise RuntimeError(f'Unknown ROS parameter type {value.type}')
    field = fields[value.type]
    result = getattr(value, field) if field else None
    return list(result) if value.type >= 5 else result


class Archive:
    def __init__(self, folder):
        self.root = Path(folder)/'reproducibility'
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root/'manifest.json'
        self.manifest = (json.loads(self.path.read_text()) if self.path.exists() else
                         dict(schema_version=SCHEMA_VERSION, created_utc=datetime.now(timezone.utc).isoformat(),
                              files=[], snapshots=[], limitations=[
                                  'Physical object, fixture, surface condition and external disturbances require operator notes.',
                                  'Robot/fixture settings not exposed by ROS or RobotState and the flashed ESP32 firmware identity require operator notes.',
                                  'ROS parameters and state are sampled between motions, not atomically across nodes.',
                                  'Source at recording time is distinct from the controller build source.']))

    def save(self):
        write_json(self.path, self.manifest)

    def store_bytes(self, data, source, role, expected=None):
        digest = hashlib.sha256(data).hexdigest()
        if expected is not None and digest != expected:
            raise RuntimeError(f'{role} SHA-256 mismatch: {source}')
        name = Path(source).name or 'artifact'
        relative = Path('files')/digest/name
        path = self.root/relative
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            path.write_bytes(data)
        elif hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise RuntimeError(f'Archived artifact was modified: {path}')
        entry = dict(source_path=str(source), role=role, sha256=digest, size_bytes=len(data),
                     archive_path=str(relative))
        if entry not in self.manifest['files']:
            self.manifest['files'].append(entry)
        return entry

    def store_file(self, path, role, expected=None):
        path = Path(path).expanduser()
        return self.store_bytes(path.read_bytes(), str(path), role, expected)

    def unpack_packets(self, state):
        packets = state.pop('artifacts', [])
        state['archived_artifacts'] = [self.store_bytes(base64.b64decode(p['data_base64'], validate=True),
            p['source_path'], p['role'], p['sha256']) for p in packets]
        return state

    def process(self, pid, boot_id, start_ticks):
        """Capture the running executable, launch args/files and application libraries."""
        proc = Path('/proc')/str(int(pid))
        actual_start=int((proc/'stat').read_text().rsplit(')',1)[1].split()[19])
        if (boot_id!=process_identity()['process_boot_id'] or int(start_ticks)!=actual_start):
            raise RuntimeError('Node process is remote or was replaced; cannot archive its local executable reliably')
        args = [x.decode(errors='replace') for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
        cwd=Path(os.readlink(proc/'cwd'))
        resolve=lambda p:Path(p) if Path(p).is_absolute() else cwd/p
        result = dict(pid=int(pid), argv=args,working_directory=str(cwd),executable_target=os.readlink(proc/'exe'),
                      executable=self.store_file(proc/'exe', 'running_executable'))
        files = []
        for i, arg in enumerate(args):
            if arg == '--params-file' and i+1 < len(args):
                files.append(args[i+1])
            elif arg.startswith('--params-file='):
                files.append(arg.split('=',1)[1])
            elif arg.endswith('.py') and resolve(arg).is_file():
                files.append(arg)
        result['launch_files'] = [self.store_file(resolve(p), 'process_launch_input') for p in files]
        maps = (proc/'maps').read_text()
        result['memory_maps'] = self.store_bytes(maps.encode(), f'process_{pid}_maps.txt', 'memory_maps')
        libraries = sorted({line.split(None,5)[-1] for line in maps.splitlines()
                            if '.so' in line and any(n in line for n in ('franka', 'eit_vmc'))
                            and not line.endswith('(deleted)')})
        result['application_libraries'] = [self.store_file(p, 'loaded_application_library') for p in libraries]
        if any('(deleted)' in line and any(n in line for n in ('franka', 'eit_vmc')) for line in maps.splitlines()):
            raise RuntimeError('A loaded robot/controller library was replaced on disk; restart before recording')
        return result


def initialize_recording(folder, config_dir=None, command=None, conditions_file=None):
    archive = Archive(folder)
    if 'environment' not in archive.manifest:
        archive.manifest['command'] = command or sys.argv
        archive.manifest['environment'] = dict(platform=platform.platform(), python=sys.version,
            operating_system=platform.freedesktop_os_release(),
            python_executable=sys.executable, working_directory=os.getcwd(),
            variables={k:os.environ[k] for k in ('ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION',
                'ROS_LOCALHOST_ONLY','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','PYTHONPATH',
                'LD_LIBRARY_PATH','EIT_CONFIG_DIR') if k in os.environ},
            python_packages=sorted((d.metadata.get('Name','') or '', d.version)
                                   for d in importlib.metadata.distributions()))
    try:
        if config_dir is not None:
            for path in sorted(Path(config_dir).rglob('*')):
                if path.is_file() and path.suffix in ('.yaml','.yml','.json','.npz','.joblib'):
                    archive.store_file(path, 'configuration_at_recording_time')
        if conditions_file:
            archive.manifest['operator_conditions'] = archive.store_file(conditions_file, 'operator_conditions')
            archive.manifest['operator_conditions_provided'] = True
        else:
            archive.manifest.setdefault('operator_conditions_provided', False)
        # Preserve the actual operator/recorder scripts even in an installed deployment.
        scripts = Path(sys.argv[0]).resolve().parent
        for name in ('experiment.py','run_block.py','joystick_feedback_node.py','plot_joystick_results.py'):
            if (scripts/name).is_file():
                archive.store_file(scripts/name, 'recording_script')
    finally:
        archive.save()
    return str(archive.path)


def ros_call(node, service, kind, request, timeout=4.):
    import rclpy
    client = node.create_client(kind, service)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('Recording service unavailable: '+service+'; rebuild/restart experiment nodes')
        future = client.call_async(request)
        deadline = time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not future.done() or future.result() is None:
            raise RuntimeError('Recording service timed out: '+service)
        return future.result()
    finally:
        node.destroy_client(client)


def all_parameters(node, target):
    from rcl_interfaces.srv import ListParameters, GetParameters
    listed = ros_call(node, target+'/list_parameters', ListParameters,
                      ListParameters.Request(prefixes=[], depth=0))
    names = sorted(listed.result.names)
    result = {}
    for offset in range(0, len(names), 64):
        batch = names[offset:offset+64]
        values = ros_call(node, target+'/get_parameters', GetParameters,
                          GetParameters.Request(names=batch)).values
        if len(values)!=len(batch):
            raise RuntimeError('Incomplete parameter response from '+target)
        result.update({k:dict(type=v.type, value=parameter_value(v)) for k,v in zip(batch,values)})
    return result


def snapshot(node, folder, tag, task, context, require_baseline=False):
    """Fail before a motion if its conditions cannot be preserved completely."""
    from std_srvs.srv import Trigger
    archive = Archive(folder)
    controller = '/eit_joystick' if task=='joystick' else '/eit_ball_roll'
    state = dict(schema_version=SCHEMA_VERSION, captured_utc=datetime.now(timezone.utc).isoformat(),
                 host_unix_ns=time.time_ns(), context=context, nodes={}, status='incomplete')
    error = None
    try:
        state['graph_nodes'] = node.get_node_names_and_namespaces()
        for target in (controller, '/eit_contact'):
            state['nodes'][target] = dict(parameters=all_parameters(node,target))
        params = state['nodes'][controller]['parameters']
        if not {'recording_process_id','recording_build_manifest','recording_process_boot_id',
                'recording_process_start_ticks'} <= params.keys():
            raise RuntimeError('Controller lacks build provenance; rebuild and restart before recording')
        build = json.loads(params['recording_build_manifest']['value'])
        state['nodes'][controller]['build'] = build
        source = build['source_archive']
        archive.store_bytes(base64.b64decode(source['data_base64'],validate=True),
                            source['path'],'controller_build_source',source['sha256'])
        try:
            state['nodes'][controller]['process'] = archive.process(params['recording_process_id']['value'],
                params['recording_process_boot_id']['value'],params['recording_process_start_ticks']['value'])
        except RuntimeError as exc:
            raise RuntimeError(f'{controller} (PID {params["recording_process_id"]["value"]}): {exc}') from exc
        targets = ['/eit_contact']
        graph = {namespace.rstrip('/')+'/'+name for name,namespace in state['graph_nodes']}
        if task=='joystick' and '/eit_joystick_shaft_feedback' in graph:
            targets.append('/eit_joystick_shaft_feedback')
        for target in targets:
            if target not in state['nodes']:
                state['nodes'][target]=dict(parameters=all_parameters(node,target))
            response = ros_call(node,target+'/recording_state',Trigger,Trigger.Request())
            if not response.success:
                raise RuntimeError('Cannot record '+target+': '+response.message)
            details = archive.unpack_packets(json.loads(response.message))
            state['nodes'].setdefault(target,{})['runtime'] = details
            try:
                state['nodes'][target]['process'] = archive.process(details['process_id'],details['process_boot_id'],
                                                                   details['process_start_ticks'])
            except RuntimeError as exc:
                raise RuntimeError(f'{target} (PID {details["process_id"]}): {exc}') from exc
        sensor = state['nodes']['/eit_contact']['runtime']
        model_files=[f for f in sensor['archived_artifacts'] if f['role']=='loaded_eit_model']
        if not model_files or model_files[0]['sha256']!=sensor.get('model_sha256'):
            raise RuntimeError('The loaded EIT model bytes/hash were not preserved')
        if require_baseline and (sensor.get('baseline') is None or sensor.get('failed')):
            raise RuntimeError('No valid calibrated EIT baseline to archive before trial')
        if task=='joystick' and (params.get('joystick_mechanism',{}).get('value')=='inward_arc'
                                or context.get('condition')=='kinematic'):
            if '/eit_joystick_shaft_feedback' not in state['nodes']:
                raise RuntimeError('Shaft feedback runtime conditions are missing')
            feedback=state['nodes']['/eit_joystick_shaft_feedback']['runtime']
            if not any(f['role']=='loaded_joystick_calibration' for f in feedback['archived_artifacts']):
                raise RuntimeError('The active shaft calibration bytes were not preserved')
        state['status'] = 'complete'
    except Exception as exc:
        state['error'] = str(exc)
        error = exc
    finally:
        path = archive.root/'snapshots'/(tag+'.json')
        write_json(path,state)
        # These YAMLs are convenient startup inputs. Runtime service state and
        # baseline are in JSON; provenance parameters must not be overridden.
        restore = {name:{'ros__parameters':{k:v['value'] for k,v in data.get('parameters',{}).items()
                   if v['type']!=0 and not k.startswith('recording_')}} for name,data in state['nodes'].items()}
        path.with_suffix('.yaml').write_text(yaml.safe_dump(restore,sort_keys=True))
        archive.manifest['snapshots'].append(dict(path=str(path.relative_to(archive.root)),status=state['status'],
                                                  context=context,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        archive.save()
    if error:
        raise RuntimeError('Experiment conditions were not fully recorded: '+str(error)) from error
    return path
