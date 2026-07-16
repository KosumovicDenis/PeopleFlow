#!/usr/bin/env python3
"""Direct rosbag extraction with the ORIGINAL risk formula (historical control).

Reproduces the pipeline that generated the May-2026 CSVs (the "good DAGs"):
  risk = exp(1/L1_distance + 1[collision]*(1/TTC + steering))
  - old cone test: P = origin - Vrel (unclamped), SAFE_DIST=2.0, OBS_SIZE=2.0
  - markovian shifts: Risk by 1 sample, subject velocity by 2, trim 2 head/tail
  - 10 Hz throttle, gaussian noise (sigma=0.05) on positions and velocities

Validated against data/scenario_tesi_1.csv (commit 3e7cc8b): the quantiles of
exp(1/L1) match (min 1.197 vs 1.198, q25 identical at 1.288); the committed
formula of that era, exp(|v_subject|), is excluded by the same statistics
(its minimum would be ~1.0). The collision bump rarely fires at these speeds.

Usage: extract_old_risk.py <bag> <subject 0|1> <out.csv> [max_duration_s]
"""
import math
import sys

import numpy as np
import rosbag
from shapely.geometry import LineString, Point, Polygon

SAFE_DIST = 2.0
OBS_SIZE = 2.0
NOISE_POS = 0.05
NOISE_VEL = 0.05


def compute_risk_old(subject, obstacle, subject_v, obstacle_v):
    risk_val = 1.0 / (abs(subject[0] - obstacle[0]) + abs(subject[1] - obstacle[1]))

    vrel = (obstacle_v[0] - subject_v[0], obstacle_v[1] - subject_v[1])
    try:
        slope_ab = (obstacle[1] - subject[1]) / (obstacle[0] - subject[0])
    except ZeroDivisionError:
        slope_ab = 0.0001
    if slope_ab == 0:
        slope_ab = 0.0001
    slope_pab = -1 / slope_ab
    dx = OBS_SIZE / (1 + slope_pab ** 2) ** 0.5
    dy = dx * slope_pab
    left = Point(obstacle[0] - dx, obstacle[1] - dy)
    right = Point(obstacle[0] + dx, obstacle[1] + dy)
    origin = Point(subject[0], subject[1])
    cone = Polygon([origin, left, right])

    # original lookahead: relative velocity, unclamped
    p = Point(origin.x - vrel[0], origin.y - vrel[1])
    dist = math.hypot(obstacle[0] - subject[0], obstacle[1] - subject[1])
    collision = p.within(cone) and dist < SAFE_DIST

    if collision:
        v_rel_norm = math.hypot(*vrel)
        if v_rel_norm > 0:
            ttc = dist / v_rel_norm
            steering = min(p.distance(LineString([origin, left])),
                           p.distance(LineString([origin, right])))
            risk_val = risk_val + 1 / ttc + steering

    return math.exp(min(risk_val, 700.0))   # overflow guard only


def main():
    bagpath, subject_mode, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    max_duration = float(sys.argv[4]) if len(sys.argv) > 4 else 1e12

    rows = []
    last_t, first_t = 0.0, None
    rng = np.random.default_rng()
    bag = rosbag.Bag(bagpath)
    for _, msg, _ in bag.read_messages(topics=['/pedsim_simulator/simulated_agents']):
        t = msg.header.stamp.to_sec()
        if first_t is None:
            first_t = t
        if t - first_t > max_duration:
            break
        if t - last_t < 0.099:
            continue
        last_t = t

        state = {}
        for a in msg.agent_states:
            if a.id in (0, 1):
                state[a.id] = (np.array([a.pose.position.x, a.pose.position.y]),
                               np.array([a.twist.linear.x, a.twist.linear.y]))
        if len(state) < 2:
            continue

        noisy = {i: (p + rng.normal(0, NOISE_POS, 2), v + rng.normal(0, NOISE_VEL, 2))
                 for i, (p, v) in state.items()}
        h_p, h_v = noisy[0]
        r_p, r_v = noisy[1]

        if subject_mode == 1:
            s_p, s_v, o_p, o_v = r_p, r_v, h_p, h_v
        else:
            s_p, s_v, o_p, o_v = h_p, h_v, r_p, r_v

        risk = compute_risk_old(s_p, o_p, s_v, o_v)
        rows.append([t, float(np.linalg.norm(r_v)), float(np.linalg.norm(h_v)), risk])
    bag.close()

    # original markovian shifts: Risk by 1, subject velocity by 2, trim 2+2
    data = np.array(rows)
    data[1:, 3] = data[:-1, 3]                      # Risk shift 1
    vcol = 1 if subject_mode == 1 else 2            # Vr if robot subject else Vh
    data[2:, vcol] = data[:-2, vcol]                # subject velocity shift 2
    data = data[2:-2]

    with open(out, 'w') as f:
        f.write('Timestamp,Vr,Vh,Risk\n')
        for r in data:
            f.write(f'{r[0]:.3f},{r[1]:.6f},{r[2]:.6f},{r[3]:.6f}\n')
    print(f'{out}: {len(data)} rows')


if __name__ == '__main__':
    main()
