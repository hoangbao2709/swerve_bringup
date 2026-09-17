#!/usr/bin/env python3
"""Aggregate TEST-ONLY caster-trail yaw sweeps without treating them as CAD."""
import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SWEEP = ROOT / 'artifacts' / 'caster_trail_sweep'
TRIALS = ((0, 'trail_000mm'), (20, 'trail_020mm'), (30, 'trail_030mm'), (40, 'trail_040mm'))


def load(path):
    with open(path, encoding='utf-8') as stream: return json.load(stream)


def main():
    rows, detailed = [], {}
    for magnitude_mm, directory in TRIALS:
        cases = {}
        for case in ('yaw+', 'yaw-'):
            path = SWEEP / directory / 'cases' / case / 'caster_alignment_summary.json'
            if not path.exists(): continue
            report = load(path)
            cases[case] = report
            row = {'trail_test_value_m': magnitude_mm / 1000.0, 'trail_test_value_mm': magnitude_mm,
                   'case': case, 'classification': 'NOT PHYSICAL CALIBRATION',
                   'integrated_gt_yaw': report['integrated_gt_yaw'],
                   'integrated_encoder_yaw': report['integrated_encoder_yaw'],
                   'integrated_odom_yaw': report['integrated_odom_yaw'],
                   'integrated_imu_yaw': report['integrated_imu_yaw'],
                   'encoder_to_gt_yaw_ratio': report['encoder_to_gt_yaw_ratio'],
                   'stall_time': report['t_stall'], 'drive_load_share': report['drive_load_share'],
                   'direction_correct': report['direction_correct'],
                   'physics_acceptance_pass': report['physics_acceptance_pass']}
            for caster, values in report['casters'].items():
                row[f'{caster}_mean_heading_error'] = values['mean_equivalent_heading_error']
                row[f'{caster}_p95_heading_error'] = values['p95_heading_error']
                row[f'{caster}_swivel_travel'] = values['swivel_travel']
                row[f'{caster}_mean_roll_velocity'] = values['mean_abs_roll_velocity']
            rows.append(row)
        detailed[f'{magnitude_mm}mm'] = cases
    output = {'classification': 'NOT PHYSICAL CALIBRATION', 'real_measurement_required': True,
              'sign_rule': 'local -X axle offset is trailing for local +X rolling direction',
              'trials': detailed,
              'acceptance_target': 'abs(encoder_to_gt_yaw_ratio - 1.0) <= 0.15'}
    with open(SWEEP / 'caster_trail_sweep_summary.json', 'w', encoding='utf-8') as stream:
        json.dump(output, stream, indent=2)
    if rows:
        with open(SWEEP / 'caster_trail_sweep_summary.csv', 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator='\n')
            writer.writeheader(); writer.writerows(rows)
    print(json.dumps({'completed_cases': len(rows), 'classification': output['classification']}, indent=2))


if __name__ == '__main__': main()
