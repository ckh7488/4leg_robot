"""Run one reproducible checkpoint; CAD and simulation use separate Python runtimes."""
from pathlib import Path
import argparse, datetime, hashlib, importlib.metadata, json, platform, subprocess, sys
ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--freecad-python', help='FreeCAD Python executable; omit to check committed simulation assets only')
    parser.add_argument('--suite', choices=['full','response'], default='full')
    parser.add_argument('--require-response-targets', action='store_true')
    args = parser.parse_args()
    commands = []
    if args.freecad_python and args.suite=='full':
        commands += [(args.freecad_python, 'scripts/build_cad.py', []),
                     (args.freecad_python, 'scripts/check_cad.py', [])]
        commands += [(args.freecad_python, 'scripts/check_motion.py', ['--leg', leg]) for leg in ['FL','FR','RL','RR']]
        commands += [(sys.executable, 'scripts/check_failure_gates.py', ['--freecad-python',args.freecad_python]),
                     (args.freecad_python, 'scripts/export_dynamics.py', [])]
    scripts = (['scripts/build_models.py','scripts/check_model_contract.py','scripts/check_sim.py'] if args.suite=='full' else [])
    scripts += ['scripts/check_response_regression.py','scripts/check_step_response.py','scripts/check_gain_sweep.py']
    if args.suite=='full': scripts += ['scripts/package_print.py','scripts/render_cad.py']
    commands += [(sys.executable, script, ['--require-targets'] if args.require_response_targets and script.endswith('check_step_response.py') else []) for script in scripts]
    logdir = ROOT / '_runs'
    logdir.mkdir(exist_ok=True)
    results = []
    for i, (python, script, extra) in enumerate(commands):
        result = subprocess.run([python, str(ROOT / script), *extra], cwd=ROOT,
                                capture_output=True, text=True, encoding='utf-8', errors='replace')
        (logdir / f'{i:02d}-{Path(script).stem}.log').write_text(result.stdout + result.stderr, encoding='utf-8')
        recorded_extra=['<FreeCAD Python>' if x==args.freecad_python else x for x in extra]
        results.append({'script': script, 'args': recorded_extra, 'exit_code': result.returncode})
        print(f'{script} {" ".join(extra)}: exit {result.returncode}', flush=True)
        if result.returncode:
            print(result.stderr[-4000:] or result.stdout[-4000:], flush=True)
            break
    inputs = [ROOT/'design.json', *sorted((ROOT/'scripts').glob('*.py')),
              *sorted((ROOT/'control').glob('*.py')), *sorted(ROOT.glob('requirements-sim*.txt'))]
    artifacts = [ROOT/'cad/microquad-xl330-v1.FCStd', *sorted((ROOT/'cad').glob('*.json')),
                 ROOT/'models/dynamics.json', ROOT/'models/mujoco/robot.xml',
                 ROOT/'models/urdf/robot.urdf', ROOT/'print/manifest.json',
                 *sorted((ROOT/'models/meshes').glob('*.stl')), *sorted((ROOT/'print/stl').glob('*.stl')),
                 *sorted((ROOT/'print/3mf').glob('*.3mf')),
                 *sorted(p for p in (ROOT/'reports').glob('*.json') if p.name!='checkpoint-validation.json')]
    def hashes(paths):
        def digest(p):
            if not p.exists(): return None
            data=p.read_bytes()
            if p.suffix.lower() not in {'.fcstd','.step','.stl','.3mf','.png'}: data=data.replace(b'\r\n',b'\n')
            return hashlib.sha256(data).hexdigest()
        return {p.relative_to(ROOT).as_posix(): digest(p) for p in paths}
    versions = {}
    for name in ['numpy','mujoco','better-actuator-models','colorama','Pillow','trimesh']:
        try: versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError: versions[name] = 'unavailable'
    report = {
        'scope': 'Executed checks only; no hardware, walking, RL or Isaac claim',
        'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'python_version': platform.python_version(), 'platform': platform.system(),
        'hash_encoding': 'Text LF-normalized; binary CAD/mesh/package/image raw bytes',
        'package_versions': versions, 'suite': args.suite,
        'cad_regenerated': bool(args.freecad_python) and args.suite=='full',
        'response_numerical_targets_met': json.loads((ROOT/'reports/step-response.json').read_text(encoding='utf-8')).get('all_single_trial_targets_met') if any(r['script']=='scripts/check_step_response.py' and r['exit_code'] in [0,1] for r in results) and (ROOT/'reports/step-response.json').exists() else None,
        'response_p95_verified': False,
        'commands': results, 'all_commands_passed': len(results)==len(commands) and all(r['exit_code']==0 for r in results),
        'input_sha256': hashes(inputs), 'artifact_sha256': hashes(artifacts)}
    (ROOT/'reports/checkpoint-validation.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return 0 if report['all_commands_passed'] else 1

if __name__ == '__main__':
    sys.exit(main())
