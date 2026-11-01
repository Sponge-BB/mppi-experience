#!/usr/bin/env python3
"""Equal-input native feasibility continuation; no simulator or control output.

The captured iterate was reached after part of the original request budget had
already elapsed. Both branches receive only that request's unspent budget.
"""
import argparse
import json
from pathlib import Path

import replay_window_continuity as previous

HERE = Path(__file__).resolve().parent


def prepare(trial, output):
    value = previous.prepare(trial, output)
    capture = value['capture']
    value.update(schema='feasibility_continuation_saved_input_v1',
        budget_contract=dict(original_request_budget_s=capture['shared_budget_s'],
            already_spent_s=capture['original_elapsed_s'],
            continuation_budget_s=capture['original_remaining_budget_s'],
            deadline_reset_between_stages=False,
            final_gate_counts_toward_delivery_deadline=True,
            setup_scope='one shared scale preparation outside both continuation budgets'),
        purpose='Same captured accepted iterate and remaining request budget; late versus early identical failed-piece weight10 continuation with continuous dynamic objective; no online performance inference')
    output.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    return value


def recipe(build, output_directory):
    value = previous.recipe(build, output_directory)
    # Reuse the exact old fixture definitions while resolving their includes
    # against this child build, whose native verdict adds collision metadata.
    # Including the old .cpp directly would make quoted includes resolve to the
    # original parent directory and give the replay an outdated verdict layout.
    fixture_source = HERE / 'window_continuity_replay.cpp'
    fixture_text = fixture_source.read_text().split('\nvoid mechanismChecks(', 1)
    if len(fixture_text) != 2:
        raise ValueError('Missing unique old fixture boundary')
    output_directory.mkdir(parents=True, exist_ok=True)
    fixture_header = output_directory / 'feasibility_replay_fixture.hpp'
    fixture_header.write_text(fixture_text[0] + '\n')
    source = HERE / 'feasibility_continuation_replay.cpp'
    obj = output_directory / 'feasibility_continuation_replay.o'
    target = output_directory / 'feasibility_continuation_replay'
    command = value['compile']
    command.insert(1, '-I' + str(output_directory))
    command[command.index('-c') + 1] = str(source)
    command[command.index('-o') + 1] = str(obj)
    link = value['link']
    link[1] = str(obj)
    link[link.index('-o') + 1] = str(target)
    value.update(binary=str(target), source_sha256=previous.sha(source),
        reused_fixture_sha256=previous.sha(fixture_source),
        generated_fixture_path=str(fixture_header), generated_fixture_sha256=previous.sha(fixture_header))
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='action', required=True)
    p = subs.add_parser('prepare')
    p.add_argument('--trial', type=Path, default=previous.TRIAL)
    p.add_argument('--output', type=Path, required=True)
    p = subs.add_parser('recipe')
    p.add_argument('--build', type=Path, required=True)
    p.add_argument('--output-directory', type=Path, required=True)
    p = subs.add_parser('run')
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--master-uri', required=True)
    p.add_argument('--mode', choices=('baseline', 'repair', 'both'), default='both')
    args = parser.parse_args()
    if args.action == 'prepare':
        value = prepare(args.trial.resolve(), args.output.resolve())
        print(json.dumps(dict(input=str(args.output.resolve()),
            pieces=value['pieces'], gears=value['gears'], budget_contract=value['budget_contract'])))
    elif args.action == 'recipe':
        print(json.dumps(recipe(args.build.resolve(), args.output_directory.resolve()), indent=2))
    else:
        previous.run(args.binary.resolve(), args.input.resolve(), args.output.resolve(),
            args.master_uri, args.mode)


if __name__ == '__main__':
    main()
