#!/usr/bin/env python3
"""Rerun the verified native-contact S1 recipe in a fresh result directory."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--gui', action='store_true', help='Run live Isaac Kit with wall-paced viewport updates; physics is computed again.')
parser.add_argument('--output-dir', type=Path, help='Fresh result directory; existing directories are refused.')
options = parser.parse_args()
base = ROOT/'artifacts/bimanual/planning/so101_surface_matched_pad_20260906'
recipe = json.loads((base/'validated_native_recipe.json').read_text())
for filename, digest in recipe['input_sha256'].items():
    path = Path(filename)
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
        raise SystemExit(f'Verified input changed: {path}; revalidate before replay.')
out = (options.output_dir or base/'grasp_review'/('native_replay_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))).resolve()
out.mkdir(parents=True, exist_ok=False)
argv = list(recipe['argv'])
argv[argv.index('--output')+1] = str(out/'result.json')
if options.gui:
    argv = [value for value in argv if value != '--headless']
    if '--simulation-render-interval' in argv:
        argv[argv.index('--simulation-render-interval')+1] = '1'
    else:
        argv += ['--simulation-render-interval', '1']
    if '--live-render-pacing' not in argv:
        argv += ['--live-render-pacing']
    argv += ['--viz', 'kit', '--keep-open']
(out/'argv.json').write_text(json.dumps(argv, indent=2)+'\n')
env = dict(os.environ)
env.update(recipe.get('execution_environment', {}))
if options.gui:
    env.setdefault('DISPLAY', ':1')
print(f'Results and log: {out}', flush=True)
started = time.monotonic()
with (out/'isaac.log').open('w') as log:
    result = subprocess.run(argv, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
if not (out/'result.json').exists():
    raise SystemExit(f'Simulation did not produce a complete result; inspect {out/"isaac.log"}')
(out/'execution.json').write_text(json.dumps({'process_exit': result.returncode,
    'result_exists': True, 'elapsed_s': time.monotonic()-started,
    'elapsed_includes_gui_review': options.gui}, indent=2)+'\n')
subprocess.run([argv[0], str(base/'grasp_review/verify_native_fold.py'), str(out)], check=True)
print(json.loads((out/'result.json').read_text())['status'])
raise SystemExit(result.returncode)
