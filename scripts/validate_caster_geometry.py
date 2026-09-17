#!/usr/bin/env python3
"""Gate final physical-caster acceptance on a measured axle-offset vector."""
import argparse
import json
from pathlib import Path

import yaml


def known_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/caster_geometry.yaml')
    parser.add_argument('--final-acceptance', action='store_true',
                        help='enforce a real measured caster trail; diagnostic mode remains allowed at zero')
    parser.add_argument('--json-output')
    args = parser.parse_args()
    with open(args.config, encoding='utf-8') as stream:
        caster = yaml.safe_load(stream)['caster']
    raw = caster.get('measured_values_mm', {})
    raw_complete = all(known_number(raw.get(name)) for name in
                       ('front_left', 'front_right', 'rear_left', 'rear_right'))
    offset_x_known = known_number(caster.get('axle_offset_x_m'))
    offset_y_known = known_number(caster.get('axle_offset_y_m'))
    real_trail_available = offset_x_known and offset_y_known and raw_complete and \
        caster.get('source') not in (None, '', 'measurement_required')
    result = {
        'real_trail_required': bool(args.final_acceptance),
        'real_trail_available': real_trail_available,
        'current_model_trail_m': 0.0,
        'configured_axle_offset_x_m': caster.get('axle_offset_x_m'),
        'configured_axle_offset_y_m': caster.get('axle_offset_y_m'),
        'raw_measurements_complete': raw_complete,
        'status': 'PASS' if not args.final_acceptance or real_trail_available else 'REAL_CASTER_TRAIL_REQUIRED',
    }
    if args.json_output:
        Path(args.json_output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json_output).write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
