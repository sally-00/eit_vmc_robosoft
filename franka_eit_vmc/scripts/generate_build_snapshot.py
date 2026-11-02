#!/usr/bin/env python3
"""Embed the exact package build inputs, including dirty/untracked source, in the controller."""
import argparse
import base64
import gzip
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile


def generate(source, output, build_dir, settings=None):
    source, output, build_dir = Path(source), Path(output), Path(build_dir)
    files = {}
    for name in ('src','include','scripts','python','msg','config','launch','models','test'):
        for path in (source/name).rglob('*'):
            if path.is_file() and not any(p in ('__pycache__','.pytest_cache') for p in path.parts):
                files[str(Path('franka_eit_vmc')/path.relative_to(source))] = path.read_bytes()
    for name in ('CMakeLists.txt','package.xml','README.md','VALIDATION.md','LICENSE','NOTICE'):
        if (source/name).is_file():
            files['franka_eit_vmc/'+name] = (source/name).read_bytes()
    for path in (source.parent/'joystick_hardware').rglob('*'):
        if path.is_file() and path.suffix in ('.py','.sh','.ino','.md') and 'recordings' not in path.parts:
            files[str(Path('joystick_hardware')/path.relative_to(source.parent/'joystick_hardware'))] = path.read_bytes()
    def git(*args):
        result = subprocess.run(['git','-C',str(source),*args],capture_output=True,timeout=10.)
        return result.stdout.decode(errors='replace') if result.returncode==0 else None
    revision = git('rev-parse','HEAD')
    status = git('status','--porcelain','--','.','../joystick_hardware')
    diff = git('diff','--binary','HEAD','--','.','../joystick_hardware')
    files['build/git_diff.patch'] = (diff or '').encode()
    for name in ('CMakeCache.txt','compile_commands.json'):
        if (build_dir/name).is_file():
            files['build/'+name] = (build_dir/name).read_bytes()
    hashes = {p:hashlib.sha256(data).hexdigest() for p,data in sorted(files.items())}
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer,mode='w') as archive:
        for name,data in sorted(files.items()):
            info=tarfile.TarInfo(name);info.size=len(data);info.mode=0o644;info.mtime=0
            archive.addfile(info,io.BytesIO(data))
    compressed = gzip.compress(buffer.getvalue(),mtime=0)
    manifest = dict(schema_version=1,build_settings=settings or {},git_commit=revision.strip() if revision else None,
                    git_status=status,build_input_sha256=hashes,
                    source_archive=dict(path='controller_build_source.tar.gz',
                        sha256=hashlib.sha256(compressed).hexdigest(),
                        data_base64=base64.b64encode(compressed).decode()))
    output.mkdir(parents=True,exist_ok=True)
    encoded=json.dumps(manifest,sort_keys=True,separators=(',',':'))
    header='#pragma once\nnamespace franka_eit_vmc {\ninline constexpr const char* recording_build_manifest = R"EIT_BUILD('+encoded+')EIT_BUILD";\n}\n'
    for name,data in (('recording_build.hpp',header.encode()),('manifest.json',(encoded+'\n').encode()),
                      ('controller_build_source.tar.gz',compressed)):
        path=output/name
        if not path.exists() or path.read_bytes()!=data:
            path.write_bytes(data)
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--build-dir',type=Path,required=True)
    parser.add_argument('--compiler',default='')
    parser.add_argument('--dependency-versions',default='')
    args=parser.parse_args();generate(args.source,args.output,args.build_dir,
        dict(compiler=args.compiler,dependency_versions=args.dependency_versions))
