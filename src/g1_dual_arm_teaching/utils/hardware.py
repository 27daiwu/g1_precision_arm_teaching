"""Explicit human review gates; never infer waist topology from motor telemetry."""
import numpy as np


def weight_target(value):
    if isinstance(value, bool) or not np.isfinite(value) or not 0 < value <= 1:
        raise ValueError('weight must satisfy 0 < weight <= 1')
    return float(value)


def validate_variant(config):
    r, w = config['robot'], config['waist']
    if r['model'] != 'G1' or r['dof'] != 29:
        raise ValueError('only explicitly configured G1 29DoF is supported')
    active, locked = w['active_joints'], w['locked_joints']
    if not isinstance(active, list) or not isinstance(locked, list):
        raise ValueError('waist joint lists required')
    if any(type(i) is not int or i not in (12, 13, 14) for i in active + locked):
        raise ValueError('invalid waist motor id')
    if len(set(active + locked)) != len(active + locked):
        raise ValueError('duplicate/overlapping waist joints')
    kind = w['configuration']
    if kind not in ('UNKNOWN', 'ACTIVE_3DOF', 'YAW_ONLY', 'LOCKED_VARIANT'):
        raise ValueError('invalid waist configuration')
    if type(w['configuration_verified']) is not bool:
        raise ValueError('waist verification must be boolean')
    if kind == 'UNKNOWN' and (active or locked or w['configuration_verified']):
        raise ValueError('UNKNOWN waist cannot be verified or assigned joints')
    if kind != 'UNKNOWN':
        if set(active + locked) != {12, 13, 14}:
            raise ValueError('waist partition must cover 12/13/14')
        if kind == 'ACTIVE_3DOF' and set(active) != {12, 13, 14}:
            raise ValueError('ACTIVE_3DOF requires all waist joints')
        if kind == 'YAW_ONLY' and (active != [12] or set(locked) != {13, 14}):
            raise ValueError('YAW_ONLY requires active 12 and locked 13/14')
        if kind == 'LOCKED_VARIANT' and not {13, 14}.issubset(locked):
            raise ValueError('LOCKED_VARIANT requires explicit locked 13/14')
    if w['experiment'] not in ('SEND_ACQUIRE_REFERENCE', 'ZERO_GAIN_NO_COMMAND'):
        raise ValueError('invalid waist experiment')


def active_waist(config):
    w = config['waist']
    if not w['send_commands'] or w['experiment'] == 'ZERO_GAIN_NO_COMMAND':
        return []
    return w['active_joints'] if w['configuration_verified'] else []


def verified_limits(config):
    metadata = config['metadata']
    if metadata['hardware_verified'] is not True or not metadata['source'] or metadata['source'] == 'UNVERIFIED':
        raise RuntimeError('unverified joint limits metadata')
    entries = list(config['joints'].values())
    ids = [j['motor_id'] for j in entries]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate limit motor ids')
    result = {}
    for motor in list(range(15, 29)) + active_waist(config):
        matches = [j for j in entries if j['motor_id'] == motor]
        if len(matches) != 1 or matches[0]['verified'] is not True:
            raise RuntimeError(f'unverified joint limit: motor {motor}')
        j = matches[0]
        if j['min'] is None or j['max'] is None or not np.isfinite([j['min'], j['max']]).all() or j['min'] >= j['max']:
            raise ValueError(f'invalid joint limits: motor {motor}')
        result[motor] = (j['min'], j['max'])
    return result


def require_real_acquire(config, current_pose_only):
    validate_variant(config)
    r, w = config['robot'], config['waist']
    if r['model_confirmed'] is not True or r['arm_joint_mapping_verified'] is not True:
        raise RuntimeError('robot model and arm mapping must be manually verified')
    if w['configuration_verified'] is not True or w['configuration'] == 'UNKNOWN':
        raise RuntimeError('waist configuration unverified')
    if not current_pose_only:
        verified_limits(config)
        if r['ready_for_phase1_real_joint_motion'] is not True:
            raise RuntimeError('Phase 1 real joint motion has not been manually approved')


def require_teaching_topology(config):
    """Validate the robot mapping shared by Teaching and Replay DDS entrypoints."""
    robot, waist = config['robot'], config['waist']
    if robot['domain_id'] != 0:
        raise ValueError('Golden hardware/mapping prerequisites not met')
    if not all(robot[key] is True for key in
               ('hardware_reviewed', 'model_confirmed', 'arm_joint_mapping_verified')):
        raise ValueError('Golden hardware/mapping prerequisites not met')
    if waist['configuration_verified'] is not True or waist['active_joints'] != [12, 13, 14]:
        raise ValueError('Golden hardware/mapping prerequisites not met')


def resolve_network_interface(config, requested=None):
    """Use the configured interface or require one unambiguous live IPv4 link."""
    if requested:
        return requested
    configured = config['robot'].get('network_interface')
    if configured:
        return configured
    try:
        from psutil import net_if_addrs, net_if_stats
        import socket
    except ImportError as exc:
        raise RuntimeError('network_interface is unset and psutil is unavailable; set it in configs/robot.yaml') from exc
    addresses, stats = net_if_addrs(), net_if_stats()
    candidates = []
    for name, entries in addresses.items():
        if name == 'lo' or name not in stats or not stats[name].isup:
            continue
        if any(item.family == socket.AF_INET and not item.address.startswith('127.')
               for item in entries):
            candidates.append(name)
    if len(candidates) != 1:
        raise RuntimeError(f'cannot uniquely identify robot network interface; candidates={sorted(candidates)}; set robot.network_interface in configs/robot.yaml')
    return candidates[0]
