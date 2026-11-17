#!/usr/bin/python3
"""Locally enable/restore CAN listening; never open a CAN socket or send frames.

Run as the desktop user. Only the two ip-link operations request sudo.
The original Piper setup documents a 1,000,000 bit/s bus.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time


ROOT = Path(__file__).resolve().parent
RECORD = ROOT / 'can_receive_only_state.json'
IP = '/usr/sbin/ip'


def inspect(interface):
    result = subprocess.check_output([IP, '-json', '-details', 'link', 'show', interface])
    items = json.loads(result)
    if len(items) != 1 or items[0].get('linkinfo', {}).get('info_kind') != 'can':
        raise SystemExit('Expected the already identified CAN device; refusing any other interface.')
    return items[0]


def privileged(interface, *args):
    subprocess.run(['/usr/bin/sudo', IP, 'link', 'set', interface, *args], check=True)


def save(value):
    temp = RECORD.with_suffix('.tmp')
    temp.write_text(json.dumps(value, indent=2) + '\n')
    temp.replace(RECORD)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['enable', 'enable-ack', 'restore'])
    parser.add_argument('--interface')
    args = parser.parse_args()
    previous = json.loads(RECORD.read_text()) if RECORD.exists() else {}
    interfaces = json.loads(subprocess.check_output([IP, '-json', '-details', 'link', 'show']))
    if args.interface:
        interface = args.interface
    elif previous.get('before', {}).get('ifindex') is not None:
        matches = [item['ifname'] for item in interfaces
                   if item['ifindex'] == previous['before']['ifindex']]
        if len(matches) != 1:
            raise SystemExit('Recorded CAN interface is missing; no interface changed.')
        interface = matches[0]
    else:
        matches = [item['ifname'] for item in interfaces
                   if item.get('linkinfo', {}).get('info_kind') == 'can']
        if len(matches) != 1:
            raise SystemExit('Expected exactly one CAN interface; specify --interface explicitly.')
        interface = matches[0]
    before = inspect(interface)
    if args.action == 'enable':
        if 'UP' in before['flags']:
            raise SystemExit(interface + ' is already UP; inspect its owner before changing it. No change made.')
        record = {'before': before, 'started_wall': time.time(), 'state': 'preparing',
                  'bitrate_source': 'src/piper_ros/can_activate.sh:7,123-125',
                  'frames_sent_by_this_program': 0}
        save(record)
        # Fail closed if this hardware does not support listen-only. Never retry
        # in normal mode and never start the SDK/automatic arm enabling.
        privileged(interface, 'type', 'can', 'bitrate', '1000000', 'listen-only', 'on')
        privileged(interface, 'up')
        after = inspect(interface)
        detail = subprocess.check_output([IP, '-details', 'link', 'show', interface]).decode()
        if 'LISTEN-ONLY' not in detail:
            privileged(interface, 'down')
            record.update(state='failed_verification_down', after=inspect(interface), detail=detail)
            save(record)
            raise SystemExit('LISTEN-ONLY was not confirmed; interface has been returned DOWN.')
        record.update(state='listening', after=after, detail=detail)
        save(record)
        print(detail)
        print('CAN receive-only interface ready. No arm enable or motion command was sent.')
    elif args.action == 'enable-ack':
        record = previous
        if not record or record.get('before', {}).get('ifindex') != before['ifindex']:
            raise SystemExit('No matching prior listen-only setup; refusing to alter interface.')
        # Some point-to-point buses have no other node to acknowledge feedback.
        # Normal controller mode supplies hardware ACK bits. It does not create
        # application data frames, enable a motor, or send an arm command.
        record['before_ack_mode'] = before
        save(record)
        privileged(interface, 'down')
        privileged(interface, 'type', 'can', 'bitrate', '1000000', 'listen-only', 'off')
        privileged(interface, 'up')
        after = inspect(interface)
        record.update(state='application_receive_only_hardware_ack', after=after,
                      ack_mode_wall=time.time(), current_interface=interface)
        save(record)
        print(subprocess.check_output([IP, '-details', 'link', 'show', interface]).decode())
        print('CAN hardware ACK enabled. No SDK, arm enable or motion command was sent.')
    else:
        record = previous
        if not record or record.get('before', {}).get('ifindex') != before['ifindex']:
            raise SystemExit('No matching interface setup record; refusing to alter interface.')
        privileged(interface, 'down')
        privileged(interface, 'type', 'can', 'listen-only', 'off')
        record.update(state='restored_down', restored_wall=time.time(), restored=inspect(interface))
        save(record)
        print(interface + ' is DOWN; listen-only is OFF. Bitrate remains the documented 1 Mbit/s default.')


if __name__ == '__main__':
    main()
