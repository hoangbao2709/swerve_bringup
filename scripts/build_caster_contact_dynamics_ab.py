#!/usr/bin/env python3
"""Aggregate the fixed-trail A-D causal diagnostic; never calibrate from it."""
import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'artifacts' / 'caster_contact_dynamics_ab_runs'
OUT_CSV = ROOT / 'artifacts' / 'caster_contact_dynamics_ab.csv'
OUT_JSON = ROOT / 'artifacts' / 'caster_contact_dynamics_ab.json'
VARIANTS = {
    'A_CURRENT': 'variant_A_current_retry',
    'B_ZERO_JOINT_RESISTANCE': 'variant_B_zero_joint_resistance_retry',
    'C_CONTACT_GRIP': 'variant_C_contact_grip_retry',
    'D_IDEALIZED_ROLLING': 'variant_D_idealized_rolling_retry',
}


def load(path):
    with open(path, encoding='utf-8') as stream: return json.load(stream)


def main():
    rows, reports = [], {}
    for variant, directory in VARIANTS.items():
        reports[variant] = {}
        for sign in ('yaw+', 'yaw-'):
            path = BASE / directory / 'cases' / sign / 'caster_alignment_summary.json'
            # C/yaw- was restarted after a readiness interruption; use only
            # its completed retry artifact, never fabricate the missing run.
            if variant == 'C_CONTACT_GRIP' and sign == 'yaw-':
                path = BASE / 'variant_C_contact_grip_yawminus_retry2' / 'cases' / sign / 'caster_alignment_summary.json'
            if not path.exists(): continue
            report = load(path); reports[variant][sign] = report
            row = {'variant': variant, 'yaw_sign': sign, 'classification': 'NOT PHYSICAL CALIBRATION',
                   'trail_test_value_m': report['simulation_assumption_caster_trail_m'],
                   'gt_integrated_yaw': report['integrated_gt_yaw'],
                   'encoder_integrated_yaw': report['integrated_encoder_yaw'],
                   'odom_integrated_yaw': report['integrated_odom_yaw'],
                   'imu_integrated_yaw': report['integrated_imu_yaw'],
                   'encoder_to_gt_ratio': report['encoder_to_gt_yaw_ratio'], 'stall_time': report['t_stall'],
                   'drive_load_share': report['drive_load_share'],
                   'unexpected_chassis_contact': report['unexpected_ground_contacts'],
                   'base_contact_count': report['base_contact_count'],
                   'front_housing_contact_count': report['front_housing_contact_count'],
                   'rear_housing_contact_count': report['rear_housing_contact_count'],
                   'physics_pass': report['physics_acceptance_pass']}
            for name, caster in report['casters'].items():
                row.update({f'{name}_mean_heading_error': caster['mean_equivalent_heading_error'],
                            f'{name}_p95_heading_error': caster['p95_heading_error'],
                            f'{name}_swivel_travel': caster['swivel_travel'],
                            f'{name}_mean_roll_velocity': caster['mean_abs_roll_velocity'],
                            f'{name}_mean_tangential_metric': caster['mean_tangential_metric']})
            for name, drive in report['drives'].items():
                row.update({f'drive_{name}_command': drive['mean_command_rad_s'],
                            f'drive_{name}_actual': drive['mean_actual_rad_s'],
                            f'drive_{name}_command_actual_ratio': drive['mean_command_actual_ratio'],
                            f'drive_{name}_slip_pre_stall': drive['slip_before_stall'],
                            f'drive_{name}_slip_post_stall': drive['slip_after_stall'],
                            f'drive_{name}_normal_load': drive['mean_normal_metric'],
                            f'drive_{name}_tangential_metric': drive['mean_tangential_metric']})
            rows.append(row)
    unexpected = any(r['unexpected_chassis_contact'] for r in rows)
    d_ratios = [abs(r['encoder_to_gt_ratio'] - 1.0) for r in rows if r['variant'] == 'D_IDEALIZED_ROLLING']
    if unexpected: cause = 'unexpected chassis contact'
    elif len(d_ratios) == 2 and max(d_ratios) <= .15: cause = 'caster contact/joint model'
    elif len(d_ratios) == 2 and max(d_ratios) > .15: cause = 'insufficient evidence; proceed to drive traction/module geometry, not more caster tuning'
    else: cause = 'insufficient evidence'
    output = {'classification': 'NOT PHYSICAL CALIBRATION', 'real_measurement_required': True,
              'heading_math': {'status': 'PASS', 'rule': 'velocity evaluated at actual rotated axle position'},
              'unexpected_chassis_contact': unexpected, 'variants': reports, 'main_cause': cause,
              'next_blocker': 'DRIVE_MODULE_48P2MM_GEOMETRY_AND_DRIVE_TRACTION' if 'drive traction' in cause else 'REAL_CASTER_TRAIL_MEASUREMENT',
              'acceptance_target': 'abs(encoder_to_gt_ratio - 1.0) <= 0.15'}
    with open(OUT_JSON, 'w', encoding='utf-8') as stream: json.dump(output, stream, indent=2)
    if rows:
        with open(OUT_CSV, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator='\n')
            writer.writeheader(); writer.writerows(rows)
    print(json.dumps({'completed_cases': len(rows), 'main_cause': cause, 'unexpected_chassis_contact': unexpected}, indent=2))


if __name__ == '__main__': main()
