"""Screen four proposed gesture endpoints; no video reconstruction or dynamics.

Run: python concepts/doro-layout-v2/motion_study.py
Uses the same environment as study.py. Existing V2 geometry and limits are read-only.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

import study

HERE = Path(__file__).resolve().parent


def main():
    targets = json.loads((HERE / 'motion_targets.json').read_text(encoding='utf-8'))
    base = np.array([0., 0., study.G['upright_hip_height']])
    cases = []
    keys = ['front_shoulder_roll', 'front_shoulder_pitch', 'front_elbow']
    for case in targets['engineering_screen']['cases']:
        arms = []
        x, y, z = case['paw_relative_xyz_m']
        for side in [-1, 1]:
            sx, sy, sz = study.G['shoulder_xyz']
            shoulder = base + np.array([sx, side * sy, sz])
            target = base + np.array([x, side * y, z])
            solution = study.solve_limb(shoulder, target, study.G['front_links'], np.eye(3))
            arm = {'side': side, 'reachable': solution['reachable']}
            if solution['reachable']:
                angles = solution['angles_deg'].tolist()
                arm.update(angles_deg=dict(zip(keys, angles)),
                           violations=[k for k, a in zip(keys, angles) if not study.allowed(k, a)],
                           endpoint_error_m=solution['endpoint_error_m'])
            arms.append(arm)
        cases.append({**case, 'arms': arms,
                      'both_arms_reach_and_assumed_ranges': all(a['reachable'] and not a['violations'] for a in arms),
                      'spherical_paw_surface_gap_m': 2 * abs(y) - 2 * study.G['paw_radius']})
    # Canonical JSON hashes do not depend on Windows checkout line endings.
    digest = lambda obj: hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()).hexdigest()
    report = {
        'status': 'endpoint geometry screen only; not a dance/clap/balance test',
        'design_sha256': digest(study.C), 'targets_sha256': digest(targets),
        'scope': targets['engineering_screen']['scope'],
        'angle_order': keys, 'cases': cases,
        'not_checked': ['continuous trajectories', 'all-parts collisions', 'paw surface orientation or contact force',
                        'balance and support changes', 'torque, speed, thermal load', 'independent head/waist gestures'],
    }
    (HERE / 'motion_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')
    for case in cases:
        print(case['id'], case['both_arms_reach_and_assumed_ranges'])


if __name__ == '__main__':
    main()
