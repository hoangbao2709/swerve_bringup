#!/usr/bin/env python3
"""Build the authoritative proper-caster yaw verdict from named artifacts."""
import csv
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW_ROOT = os.path.join(ROOT, 'artifacts', 'proper_caster_yaw_run', 'cases')
ALIGN_ROOT = os.path.join(ROOT, 'artifacts', 'proper_caster_yaw_alignment_run_v4', 'cases')
OUT = os.path.join(ROOT, 'artifacts')


def load_json(path):
    with open(path, encoding='utf-8') as stream:
        return json.load(stream)


def load_result(case):
    with open(os.path.join(RAW_ROOT, case, 'raw_motion_result.csv'), newline='', encoding='utf-8') as stream:
        return next(csv.DictReader(stream))


def boolean(value): return str(value).lower() == 'true'


def number_or_none(value):
    try: return float(value)
    except (TypeError, ValueError): return None


def main():
    raw = {case: load_result(case) for case in ('yaw+', 'yaw-')}
    alignment = {case: load_json(os.path.join(ALIGN_ROOT, case, 'caster_alignment_summary.json'))
                 for case in ('yaw+', 'yaw-')}
    stationary = load_json(os.path.join(OUT, 'proper_caster_stationary.json'))
    # Preserve a single, top-level trace artifact so plots can use both signs.
    rows = []
    for case in ('yaw+', 'yaw-'):
        with open(os.path.join(ALIGN_ROOT, case, 'caster_alignment.csv'), newline='', encoding='utf-8') as stream:
            rows.extend(csv.DictReader(stream))
    with open(os.path.join(OUT, 'caster_alignment.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)
    alignment_out = {'schema': 'caster-yaw-alignment-v1', 'sample_rate_hz_approx': 6.67,
                     'wheel_axis_equivalence': 'signed wrap(2*(actual-desired))/2 in [-pi/2, pi/2]',
                     'source_runs': {case: os.path.relpath(os.path.join(ALIGN_ROOT, case), ROOT) for case in alignment},
                     'cases': alignment}
    with open(os.path.join(OUT, 'caster_alignment_summary.json'), 'w', encoding='utf-8') as stream:
        json.dump(alignment_out, stream, indent=2)
    yaw = {}
    for case, values in raw.items():
        yaw[case] = {key: number_or_none(values.get(key)) for key in (
            'integrated_gt_yaw', 'integrated_encoder_yaw', 'integrated_odom_yaw',
            'integrated_imu_yaw', 'encoder_to_gt_yaw_ratio', 'odom_to_gt_yaw_ratio',
            'imu_to_gt_yaw_error', 'steady_gt_wz', 'steady_encoder_wz',
            'steady_odom_wz', 'steady_imu_wz')}
        yaw[case].update({'clock_verified': boolean(values.get('clock_verified')),
                          'direction_correct': boolean(values.get('direction_correct')),
                          'physics_acceptance_pass': boolean(values.get('physics_acceptance_pass')),
                          'yaw_magnitude_meaningful': boolean(values.get('gt_yaw_magnitude_meaningful')),
                          'source': os.path.relpath(os.path.join(RAW_ROOT, case, 'raw_motion_result.csv'), ROOT)})
    direction_pass = all(item['direction_correct'] for item in yaw.values())
    physics_pass = all(item['physics_acceptance_pass'] for item in yaw.values())
    blocker = True if all(item['zero_trail_confirmed_as_blocker'] is True for item in alignment.values()) else False
    summary = {
        'verdict': 'PROPER ZERO-TRAIL PHYSICS: FAIL',
        'STATIONARY': 'PASS' if stationary.get('pass') else 'FAIL',
        'YAW DIRECTION': 'PASS' if direction_pass else 'FAIL',
        'YAW PHYSICS': 'PASS' if physics_pass else 'FAIL',
        'yaw_acceptance': {'encoder_to_gt_ratio_target': 1.0, 'absolute_error_max': 0.15,
                           'rule': 'clock verified AND correct direction AND meaningful GT yaw AND ratio accepted'},
        'yaw': yaw,
        'caster_alignment': {case: alignment[case]['casters'] for case in alignment},
        'stall_correlation': {case: {'t_stall': alignment[case]['t_stall'], 'at_stall': alignment[case]['at_stall'],
                                     'evidence': alignment[case]['evidence']} for case in alignment},
        'zero_trail_confirmed_as_blocker': blocker,
        'real_trail_required': True,
        'real_trail_available': False,
        'current_model_trail_m': 0.0,
        'actual_trail_dimension_available': False,
        'trail_search': {'result': 'not found in workspace source/CAD/drawing search',
                         'model_source': 'urdf/swerve_base.urdf:84-87 explicitly documents a zero-trail approximation',
                         'measurement_required': 'horizontal distance from vertical swivel axis to caster wheel axle center'},
        'zero_trail_refactor_regression': {
            'xacro_parameter_parser': 'PASS', 'check_urdf': 'PASS', 'gz_sdf_p': 'PASS',
            'contact_geometry_source': 'artifacts/proper_caster_zero_trail_geometry.json',
            'gazebo_ros2_control': 'PASS',
            'startup_source': 'artifacts/caster_zero_trail_startup_regression/summary.json'},
        'test_only_trail_sweep': {
            'classification': 'NOT PHYSICAL CALIBRATION',
            'simulation_assumption_values_m': [0.0, 0.020, 0.030, 0.040],
            'source': 'artifacts/caster_trail_sweep/caster_trail_sweep_summary.json',
            'result': 'all yaw cases fail the <=15% encoder/GT target; do not select or promote a sweep value'},
        'if_real_trail_or_test_only_sensitivity_still_fails_investigate_in_order': [
            'swivel bearing friction/damping', 'caster wheel rolling friction/contact',
            'drive-wheel torque/traction', 'drive module steering-contact offset (~48.2 mm)',
            'swerve odometry geometry'],
        'model_comparison': {
            'simplified_baseline': {'yaw+_gt_integrated_yaw_rad': 0.193, 'yaw-_gt_integrated_yaw_rad': -0.178,
                                    'encoder_to_gt_yaw_ratio': {'yaw+': 3.12, 'yaw-': 3.73}},
            'simplified_frictionless': {'yaw+_gt_integrated_yaw_rad': 0.263, 'yaw-_gt_integrated_yaw_rad': -0.260,
                                        'encoder_to_gt_yaw_ratio': {'yaw+': 4.67, 'yaw-': 4.73}},
            'proper_2dof_zero_trail': {'yaw+_gt_integrated_yaw_rad': 0.127, 'yaw-_gt_integrated_yaw_rad': -0.115,
                                       'encoder_to_gt_yaw_ratio': {'yaw+': 9.34, 'yaw-': 10.14},
                                       'verdict': 'FAIL'}},
        'NEXT BLOCKER': 'REAL_CASTER_TRAIL_MEASUREMENT'
    }
    with open(os.path.join(OUT, 'proper_caster_yaw_summary.json'), 'w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2)
    # Repair the legacy run_acceptance report as well.  Its historical PASS
    # only meant correct direction because raw_motion formerly exited zero on
    # that single condition.  Retain readiness metadata but make its status
    # unambiguous for anyone opening this older artifact directly.
    legacy_root = os.path.join(OUT, 'proper_caster_yaw_run')
    legacy_json = os.path.join(legacy_root, 'summary.json')
    if os.path.exists(legacy_json):
        legacy = load_json(legacy_json)
        rows = legacy.get('runs', [])
        for row in rows:
            metrics = yaw.get(row.get('run'))
            if not metrics:
                continue
            row['direction_correct'] = metrics['direction_correct']
            row['physics_acceptance_pass'] = metrics['physics_acceptance_pass']
            row['direction_status'] = 'PASS' if metrics['direction_correct'] else 'FAIL'
            row['physics_status'] = 'PASS' if metrics['physics_acceptance_pass'] else 'FAIL'
            row['status'] = 'PASS' if metrics['physics_acceptance_pass'] else 'FAIL'
        legacy['passed'] = bool(rows) and all(row.get('status') == 'PASS' for row in rows)
        legacy['semantics'] = 'status is physics acceptance, not command-direction-only'
        with open(os.path.join(legacy_root, 'summary.csv'), 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator='\n')
            writer.writeheader(); writer.writerows(rows)
        with open(legacy_json, 'w', encoding='utf-8') as stream:
            json.dump(legacy, stream, indent=2)
    # The fresh caster probe is a telemetry evaluator, not a physics gate by
    # itself.  Its runner must therefore not retain a bare PASS beside the
    # authoritative raw-yaw physics failure above.
    probe_root = os.path.join(OUT, 'proper_caster_yaw_alignment_run_v4')
    probe_json = os.path.join(probe_root, 'summary.json')
    if os.path.exists(probe_json):
        probe = load_json(probe_json)
        rows = probe.get('runs', [])
        for row in rows:
            metrics = yaw.get(row.get('run'))
            if not metrics:
                continue
            row['direction_correct'] = metrics['direction_correct']
            row['physics_acceptance_pass'] = metrics['physics_acceptance_pass']
            row['direction_status'] = 'PASS' if metrics['direction_correct'] else 'FAIL'
            row['physics_status'] = 'PASS' if metrics['physics_acceptance_pass'] else 'FAIL'
            row['status'] = 'PASS' if metrics['physics_acceptance_pass'] else 'FAIL'
            row['status_basis'] = 'raw_motion named yaw physics metrics; caster probe is telemetry-only'
        probe['passed'] = bool(rows) and all(row.get('status') == 'PASS' for row in rows)
        probe['semantics'] = 'status is physics acceptance, not telemetry collection success'
        with open(os.path.join(probe_root, 'summary.csv'), 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator='\n')
            writer.writeheader(); writer.writerows(rows)
        with open(probe_json, 'w', encoding='utf-8') as stream:
            json.dump(probe, stream, indent=2)
    print(json.dumps({'STATIONARY': summary['STATIONARY'], 'YAW DIRECTION': summary['YAW DIRECTION'],
                      'YAW PHYSICS': summary['YAW PHYSICS'], 'zero_trail_confirmed_as_blocker': blocker}, indent=2))


if __name__ == '__main__': main()
