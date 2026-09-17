#!/usr/bin/env python3
"""Repair heading diagnostics in existing trail-sweep traces.

This uses recorded body twist, swivel angle, and configured axle offset; it
does not alter any motion, contact, or integrated-yaw measurement.
"""
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SWEEP_ROOT = ROOT / 'artifacts' / 'caster_trail_sweep'
PIVOTS = {
    'FL': (0.505, 0.195), 'FR': (0.505, -0.195),
    'RL': (-0.505, 0.195), 'RR': (-0.505, -0.195),
}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def equivalent_signed(actual, desired):
    return wrap(2.0 * (actual - desired)) / 2.0


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = (len(ordered) - 1) * fraction
    lo, hi = math.floor(index), math.ceil(index)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo)


def number(row, field):
    value = row.get(field, '')
    return float(value) if value not in ('', None) else None


def recompute_case(summary_path):
    csv_path = summary_path.with_name('caster_alignment.csv')
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    offset_x = float(summary['caster_axle_offset_x_m'])
    offset_y = float(summary['caster_axle_offset_y_m'])
    with csv_path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
        fields = list(rows[0].keys()) if rows else []

    added = []
    for label in PIVOTS:
        added += [f'{label}_pivot_x', f'{label}_pivot_y', f'{label}_axle_x', f'{label}_axle_y',
                  f'{label}_offset_x', f'{label}_offset_y', f'{label}_swivel_angle',
                  f'{label}_desired_heading', f'{label}_heading_error_signed', f'{label}_heading_error']
    for field in added:
        if field not in fields:
            fields.append(field)

    for row in rows:
        vx, vy, wz = (number(row, key) for key in ('gt_vx', 'gt_vy', 'gt_wz'))
        for label, (px, py) in PIVOTS.items():
            theta = number(row, f'{label}_swivel_position')
            for key, value in ((f'{label}_pivot_x', px), (f'{label}_pivot_y', py),
                               (f'{label}_offset_x', offset_x), (f'{label}_offset_y', offset_y),
                               (f'{label}_swivel_angle', theta)):
                row[key] = '' if value is None else value
            if None in (vx, vy, wz, theta):
                continue
            axle_x = px + math.cos(theta) * offset_x - math.sin(theta) * offset_y
            axle_y = py + math.sin(theta) * offset_x + math.cos(theta) * offset_y
            axle_vx, axle_vy = vx - wz * axle_y, vy + wz * axle_x
            desired = math.atan2(axle_vy, axle_vx) if math.hypot(axle_vx, axle_vy) > 1e-6 else None
            error = equivalent_signed(theta, desired) if desired is not None else None
            row[f'{label}_axle_x'], row[f'{label}_axle_y'] = axle_x, axle_y
            row[f'{label}_desired_heading'] = '' if desired is None else desired
            row[f'{label}_heading_error_signed'] = '' if error is None else error
            row[f'{label}_heading_error'] = '' if error is None else abs(error)

    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)

    for label in PIVOTS:
        errors = [number(row, f'{label}_heading_error') for row in rows]
        errors = [value for value in errors if value is not None]
        swivels = [number(row, f'{label}_swivel_position') for row in rows]
        swivels = [value for value in swivels if value is not None]
        desired = number(rows[-1], f'{label}_desired_heading') if rows else None
        caster = summary['casters'][label]
        caster.update({
            'desired_heading': desired,
            'actual_heading': swivels[-1] if swivels else None,
            'mean_equivalent_heading_error': sum(errors) / len(errors) if errors else None,
            'p95_heading_error': percentile(errors, .95),
            'final_heading_error': errors[-1] if errors else None,
            'swivel_travel': sum(abs(wrap(after - before)) for before, after in zip(swivels, swivels[1:])),
        })
    summary['heading_math_version'] = 'actual_rotated_axle_v1'
    summary['heading_math_recomputed_from_recorded_telemetry'] = True
    summary_path.write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    return str(summary_path)


def main():
    paths = sorted(SWEEP_ROOT.glob('trail_*mm/cases/yaw*/caster_alignment_summary.json'))
    if len(paths) != 8:
        raise SystemExit(f'expected eight sweep summaries, found {len(paths)}')
    for path in paths:
        print(recompute_case(path))


if __name__ == '__main__':
    main()
