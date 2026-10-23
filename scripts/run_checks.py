"""Run one reproducible checkpoint; CAD and simulation use separate Python runtimes."""
from pathlib import Path
import argparse, datetime, hashlib, importlib.metadata, json, platform, subprocess, sys
ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--freecad-python', help='FreeCAD Python executable; omit to check committed simulation assets only')
    args = parser.parse_args()
    commands = []
    if args.freecad_python:
        commands += [(args.freecad_python, 'scripts/build_cad.py', []),
                     (args.freecad_python, 'scripts/check_cad.py', [])]
        commands += [(args.freecad_python, 'scripts/check_motion.py', ['--leg', leg]) for leg in ['FL','FR','RL','RR']]
        commands += [(args.freecad_python, 'scripts/export_dynamics.py', [])]
    commands += [(sys.executable, script, []) for script in [
        'scripts/build_models.py', 'scripts/check_model_contract.py',
        'scripts/check_sim.py', 'scripts/check_step_response.py',
        'scripts/package_print.py', 'scripts/render_cad.py']]
    logdir = ROOT / '_runs'
    logdir.mkdir(exist_ok=True)
    results = []
    for i, (python, script, extra) in enumerate(commands):
        result = subprocess.run([python, str(ROOT / script), *extra], cwd=ROOT,
                                capture_output=True, text=True, encoding='utf-8', errors='replace')
        (logdir / f'{i:02d}-{Path(script).stem}.log').write_text(result.stdout + result.stderr, encoding='utf-8')
        results.append({'script': script, 'args': extra, 'exit_code': result.returncode})
        print(f'{script} {" ".join(extra)}: exit {result.returncode}', flush=True)
        if result.returncode:
            print(result.stderr[-4000:] or result.stdout[-4000:], flush=True)
            break
    inputs = [ROOT/'design.json', *sorted((ROOT/'scripts').glob('*.py')),
              *sorted((ROOT/'control').glob('*.py'))]
    artifacts = [ROOT/'cad/microquad-xl330-v1.FCStd', *sorted((ROOT/'cad').glob('*.json')),
                 ROOT/'models/dynamics.json', ROOT/'models/mujoco/robot.xml',
                 ROOT/'models/urdf/robot.urdf', ROOT/'print/manifest.json']
    def hashes(paths):
        def digest(p):
            data=p.read_bytes()
            if p.suffix.lower()!='.fcstd': data=data.replace(b'\r\n',b'\n')
            return hashlib.sha256(data).hexdigest()
        return {p.relative_to(ROOT).as_posix(): digest(p) for p in paths}
    versions = {name: importlib.metadata.version(name) for name in [
        'numpy','mujoco','better-actuator-models','colorama','Pillow','trimesh']}
    report = {
        'scope': 'Executed checks only; no hardware, walking, RL or Isaac claim',
        'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'python_version': platform.python_version(), 'platform': platform.system(),
        'hash_encoding': 'UTF-8/text LF-normalized; FCStd raw bytes',
        'package_versions': versions, 'cad_regenerated': bool(args.freecad_python),
        'commands': results, 'all_commands_passed': len(results)==len(commands) and all(r['exit_code']==0 for r in results),
        'input_sha256': hashes(inputs), 'artifact_sha256': hashes(artifacts)}
    (ROOT/'reports/checkpoint-validation.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return 0 if report['all_commands_passed'] else 1

if __name__ == '__main__':
    sys.exit(main())
