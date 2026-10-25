"""Find the newest trained position checkpoint in the local toolkit runs."""
import os
from pathlib import Path


MODEL_NAMES = {'position_model.npz', 'position_model_mlp.npz'}


def runs_directory():
    configured = os.environ.get('EIT_POSITION_RUNS_DIR')
    if configured:
        return Path(configured).expanduser()
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'eit_position_toolkit' / 'contact_position_runs'
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError(
        'Cannot find eit_position_toolkit/contact_position_runs; '
        'set EIT_POSITION_RUNS_DIR to that directory')


def select_model(path=None, runs_dir=None):
    if path is not None:
        selected = Path(path).expanduser()
        if not selected.is_file():
            raise FileNotFoundError(f'EIT model does not exist: {selected}')
        return selected
    root = Path(runs_dir) if runs_dir is not None else runs_directory()
    candidates = [p for p in root.glob('*/*.npz') if p.name in MODEL_NAMES and p.is_file()]
    if not candidates:
        raise FileNotFoundError(f'No trained position_model*.npz found in {root}')
    return max(candidates, key=lambda p: (p.stat().st_mtime_ns, str(p)))


def eit_config_path(path=None):
    if path is not None:
        return Path(path).expanduser()
    configured = os.environ.get('EIT_CONFIG_DIR')
    if configured:
        return Path(configured).expanduser() / 'eit.json'
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'franka_eit_vmc' / 'config' / 'eit.json'
        if candidate.is_file():
            return candidate
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('franka_eit_vmc')) / 'config' / 'eit.json'
