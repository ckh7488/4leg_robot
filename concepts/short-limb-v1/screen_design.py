"""Reproducible concept arithmetic, not robot inverse dynamics or walking validation."""
from pathlib import Path
import hashlib
import json

HERE = Path(__file__).resolve().parent


def screen(config):
    s = config['screening']
    g = s['gravity_m_s2']
    cases = []
    for mass in s['mass_sweep_kg']:
        for arm in s['vertical_reaction_moment_arms_m']:
            for supports in s['equal_load_support_counts']:
                cases.append(dict(mass_kg=mass, horizontal_moment_arm_m=arm,
                                  equal_load_supports=supports,
                                  support_force_contribution_nm=mass*g*arm/supports))
    allowance = sum(v for v in config['mass_allowance_kg'].values()
                    if isinstance(v, (float, int)))
    masses = []
    for architecture in config['architectures']:
        n = architecture['locomotion_axes']
        described = (architecture['front_limbs']*architecture['front_axes_per_limb']
                     + architecture['rear_limbs']*architecture['rear_axes_per_limb'])
        if n != described:
            raise ValueError('Inconsistent locomotion axis count')
        for ref in config['actuator_mass_references']:
            masses.append(dict(architecture=architecture['id'], module=ref['id'],
                               locomotion_axes=n, head_pitch_axes=architecture['head_pitch_axes'],
                               module_mass_total_kg=n*ref['mass_kg'],
                               non_locomotion_module_allowance_kg=allowance,
                               estimated_total_kg=n*ref['mass_kg']+allowance))
    return {
        'status': 'illustrative quasi-static and mass screening only',
        'scope': s['model_scope'],
        'static_cases': cases,
        'mass_scenarios': masses,
        'single_rigid_link_examples': [
            dict(length_m=l, tip_speed_m_s=s['illustrative_tip_speed_m_s'],
                 angular_speed_rad_s=s['illustrative_tip_speed_m_s']/l)
            for l in s['effective_link_examples_m']],
        'tether_example': {
            'force_n': s['tether_force_n'],
            'assumed_horizontal_force_height_m': s['tether_height_m'],
            'body_overturning_moment_nm': s['tether_force_n']*s['tether_height_m'],
            'if_instead_vertical_fraction_of_body_weight': s['tether_force_n']/(s['illustrative_total_mass_kg']*g),
            'note': 'Horizontal and vertical examples are separate directions, not simultaneous components.'},
        'verified': ['Configuration arithmetic and explicit assumptions only'],
        'not_verified': ['Joint kinematics', 'Motor packaging', 'Mass properties',
                         'Ground contact', 'Standing', 'Walking', 'Self-righting',
                         'Actuator dynamics', 'Thermal limits', 'Sim-to-real', 'Total cost']
    }


def main():
    # Git may normalize Windows CRLF checkouts to LF. Hash the same text everywhere.
    raw = (HERE/'design.json').read_text(encoding='utf-8').encode('utf-8')
    config = json.loads(raw)
    report = screen(config)
    report['input_sha256'] = hashlib.sha256(raw).hexdigest()
    report['input_sha256_normalization'] = 'UTF-8 text with universal newlines normalized to LF'
    (HERE/'screening.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')
    html = (HERE/'concept.template.html').read_text(encoding='utf-8')
    data = json.dumps({'config': config, 'report': report}, ensure_ascii=False).replace('</', '<\\/')
    html = html.replace('__CONCEPT_DATA__', data)
    (HERE/'concept.html').write_text(html, encoding='utf-8', newline='\n')
    print(json.dumps({'cases': len(report['static_cases']), 'mass_scenarios': report['mass_scenarios'],
                      'input_sha256': report['input_sha256'], 'walking_validated': False}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
