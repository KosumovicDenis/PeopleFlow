#!/usr/bin/env python3
"""Excite the mover agent's speed during a recording.

Periodically draws a new speed in [v_min, v_max] and applies it to the
agent that is NOT the subject (the mover), via the per-agent override
exposed by pedsim_simulator's dynamic_reconfigure. The exogenous variance
makes the causal link V_mover -> Risk identifiable and drowns the
tour-phase confounder.
"""

import random

import rospy
from dynamic_reconfigure.client import Client

if __name__ == '__main__':
    rospy.init_node('speed_excitation')

    # subject: 0 human at center, 1 robot at center -> mover is the other one
    subject = int(rospy.get_param('~subject', 0))
    v_min = float(rospy.get_param('~v_min', 1.0))
    v_max = float(rospy.get_param('~v_max', 2.5))
    period = float(rospy.get_param('~period', 30.0))
    seed = int(rospy.get_param('~seed', -1))

    if seed >= 0:
        random.seed(seed)

    mover_param = f'agent{1 - subject}_speed'
    rospy.loginfo(f'Speed excitation on {mover_param}: '
                  f'[{v_min}, {v_max}] m/s every {period}s')

    client = Client('pedsim_simulator', timeout=60)

    while not rospy.is_shutdown():
        speed = round(random.uniform(v_min, v_max), 2)
        try:
            client.update_configuration({mover_param: speed})
            rospy.loginfo(f'{mover_param} = {speed:.2f} m/s')
        except Exception as e:
            rospy.logwarn(f'reconfigure failed: {e}')
        rospy.sleep(period)
