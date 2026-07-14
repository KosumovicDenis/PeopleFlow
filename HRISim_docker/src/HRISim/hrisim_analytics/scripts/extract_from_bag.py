#!/usr/bin/env python3
"""Direct rosbag extraction (no replay): reads /pedsim_simulator/simulated_agents
and writes the metrics CSV in seconds instead of real-time playback.

Emits every risk-variable candidate in one pass, so alternative formulations
can be compared on the same recording:
  Timestamp, Vr, Vh, D, Risk, RiskSmooth, Collision
    D          noisy euclidean distance between the two agents
    Risk       current formula (clamped prox + clamped closing speed)
    RiskSmooth 1 - exp(-softplus(w_p*(d_safe/d - 1) + w_t*(-ddot)/d)),
               signed range rate, smooth everywhere (no max(0,.) kinks)

Usage: extract_from_bag.py <bag> <subject 0|1> <out.csv> [max_duration_s]
"""
import math
import sys

import numpy as np
import rosbag

SAFE_DIST = 2.3
OBS_SIZE = 1.0
W_PROX = 1.0
W_TTC = 1.0
NOISE_POS = 0.05
NOISE_VEL = 0.05

from shapely.geometry import LineString, Point, Polygon


def collision_flag(subject, obstacle, subject_v, obstacle_v):
    """Cone test (same as risk.py / extract_metrics.py)."""
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
    v_norm = math.hypot(*vrel)
    dist = math.hypot(obstacle[0] - subject[0], obstacle[1] - subject[1])
    scale = min(1.0, 0.9 * dist / v_norm) if v_norm > 0 else 1.0
    p = Point(origin.x - vrel[0] * scale, origin.y - vrel[1] * scale)
    return p.within(cone) and dist < SAFE_DIST


def risks(subject, obstacle, subject_v, obstacle_v):
    """(risk_current, risk_smooth, distance) from noisy state."""
    dx, dy = obstacle[0] - subject[0], obstacle[1] - subject[1]
    d = math.hypot(dx, dy)
    if d <= 0:
        return 1.0, 1.0, 0.0
    ux, uy = dx / d, dy / d
    vrel = (obstacle_v[0] - subject_v[0], obstacle_v[1] - subject_v[1])
    ddot = vrel[0] * ux + vrel[1] * uy          # signed range rate (<0 approaching)

    # current formula (clamped)
    v_closing = max(0.0, -ddot)
    risk_cur = 1.0 - math.exp(-(W_PROX * max(0.0, SAFE_DIST / d - 1.0)
                                + W_TTC * v_closing / d))

    # smooth formula (signed, softplus, no kinks)
    z = W_PROX * (SAFE_DIST / d - 1.0) + W_TTC * (-ddot) / d
    risk_smooth = 1.0 - math.exp(-math.log1p(math.exp(min(z, 30.0))))
    return risk_cur, risk_smooth, d


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
        if t - last_t < 0.099:          # 10 Hz throttle (same as extract_metrics)
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

        risk_cur, risk_smooth, d = risks(s_p, o_p, s_v, o_v)
        coll = collision_flag(s_p, o_p, s_v, o_v)
        rows.append((t, float(np.linalg.norm(r_v)), float(np.linalg.norm(h_v)),
                     d, risk_cur, risk_smooth, int(coll)))
    bag.close()

    with open(out, 'w') as f:
        f.write('Timestamp,Vr,Vh,D,Risk,RiskSmooth,Collision\n')
        for r in rows:
            f.write(f'{r[0]:.3f},{r[1]:.6f},{r[2]:.6f},{r[3]:.6f},'
                    f'{r[4]:.6f},{r[5]:.6f},{r[6]}\n')
    print(f'{out}: {len(rows)} rows')


if __name__ == '__main__':
    main()
