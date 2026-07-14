#!/usr/bin/env python

import rospy
import pandas as pd
import numpy as np
import os
import message_filters
from pedsim_msgs.msg import AgentStates
from hrisim_risk.msg import Risk
import math
from shapely.geometry import *

# Global constants for risk calculation (matches risk.py defaults)
SAFE_DIST = 2.3
OBS_SIZE = 1.0
W_PROX = 1.0
W_TTC = 1.0

def compute_risk(subject: Point, obstacle: Point, subject_v: Point, obstacle_v: Point):
    collision = False
    
    Vrel = Point(obstacle_v.x - subject_v.x, obstacle_v.y - subject_v.y)
    
    try:
        slope_AB = (obstacle.y - subject.y) / (obstacle.x - subject.x)
    except ZeroDivisionError:
        slope_AB = 0.0001

    if slope_AB == 0:
        slope_AB = 0.0001
    slope_PAB = -1 / slope_AB

    delta_x = OBS_SIZE / (1 + slope_PAB ** 2) ** 0.5
    delta_y = delta_x * slope_PAB
    
    left = Point(obstacle.x - delta_x, obstacle.y - delta_y)
    right = Point(obstacle.x + delta_x, obstacle.y + delta_y)
    
    cone_origin = Point(subject.x, subject.y)
    cone = Polygon([cone_origin, left, right])
    
    # The 1s-lookahead point P must fall inside the cone, whose base lies at
    # the obstacle distance: clamp the displacement so that relative speeds
    # larger than the distance cannot overshoot past the base and miss it.
    v_rel_norm = math.sqrt(Vrel.x**2 + Vrel.y**2)
    dist = subject.distance(obstacle)
    scale = min(1.0, 0.9 * dist / v_rel_norm) if v_rel_norm > 0 else 1.0
    P = Point(cone_origin.x - Vrel.x * scale, cone_origin.y - Vrel.y * scale)

    collision = P.within(cone) and dist < SAFE_DIST

    # --- old risk formula, kept for reference ---
    # risk_val = 1 / (abs(subject.x - obstacle.x) + abs(subject.y - obstacle.y))
    # if collision:
    #     if v_rel_norm > 0:
    #         time_collision_measure = dist / v_rel_norm
    #         steering_effort_measure = min(P.distance(LineString([cone_origin, left])), P.distance(LineString([cone_origin, right])))
    #         risk_val = risk_val + 1/time_collision_measure + steering_effort_measure
    # risk_val = math.exp(risk_val)

    # Risk w.r.t. the subject (the agent standing at the center): proximity
    # inside SAFE_DIST plus closing speed (inverse time-to-collision), then
    # normalized to [0, 1). Zero when the obstacle is far or moving away.
    if dist > 0:
        ux = (obstacle.x - subject.x) / dist
        uy = (obstacle.y - subject.y) / dist
        v_closing = max(0.0, -(Vrel.x * ux + Vrel.y * uy))
        risk_prox = max(0.0, SAFE_DIST / dist - 1.0)
        risk_ttc = v_closing / dist
        risk_val = 1.0 - math.exp(-(W_PROX * risk_prox + W_TTC * risk_ttc))
    else:
        risk_val = 1.0

    return risk_val, collision

class MetricsExtractor:
    def __init__(self):
        # Parameters
        self.bag_name = rospy.get_param('~bag_name', 'data_log')
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/ros_ws/src/HRISim/hrisim_analytics/data'))
        
        # subject_mode: 0 for Human subject, 1 for Robot subject
        self.subject_mode = int(rospy.get_param('~subject', 0))
        self.max_duration = float(rospy.get_param('~max_duration', 330.0))
        self.human_id = 0
        self.robot_id = 1
        
        # Noise parameters
        self.noise_sigma_pos = 0.05
        self.noise_sigma_vel = 0.05
        
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        self.output_file = os.path.join(self.output_dir, f"{self.bag_name}.csv")
        self.data_rows = []
        
        self.last_timestamp = 0.0
        self.first_timestamp = None
        self.limit_reached = False

        # Subscriber
        self.sub_agents = rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, self.callback)
        
        rospy.loginfo(f"Metrics Extractor initialized (10Hz, Max Duration: {self.max_duration}s).")
        rospy.loginfo(f"Subject: {'Robot' if self.subject_mode == 1 else 'Human'}")

    def callback(self, agents_msg):
        if self.limit_reached:
            return

        curr_time = agents_msg.header.stamp.to_sec()
        
        if self.first_timestamp is None:
            self.first_timestamp = curr_time
            
        # Check if time limit reached
        if curr_time - self.first_timestamp > self.max_duration:
            rospy.logwarn(f"Reached time limit of {self.max_duration}s. Saving and shutting down.")
            self.limit_reached = True
            rospy.signal_shutdown("Time limit reached")
            return

        # Enforce 10Hz
        if curr_time - self.last_timestamp < 0.099:
            return
        
        self.last_timestamp = curr_time

        h_pos, h_vel = None, None
        r_pos, r_vel = None, None

        for agent in agents_msg.agent_states:
            if agent.id == self.human_id:
                h_pos = np.array([agent.pose.position.x, agent.pose.position.y])
                h_vel = np.array([agent.twist.linear.x, agent.twist.linear.y])
            elif agent.id == self.robot_id:
                r_pos = np.array([agent.pose.position.x, agent.pose.position.y])
                r_vel = np.array([agent.twist.linear.x, agent.twist.linear.y])
        
        if h_pos is None or r_pos is None:
            return

        # Add Noise
        h_pos_n = h_pos + np.random.normal(0, self.noise_sigma_pos, 2)
        h_vel_n = h_vel + np.random.normal(0, self.noise_sigma_vel, 2)
        r_pos_n = r_pos + np.random.normal(0, self.noise_sigma_pos, 2)
        r_vel_n = r_vel + np.random.normal(0, self.noise_sigma_vel, 2)

        h_v_noisy = np.linalg.norm(h_vel_n)
        r_v_noisy = np.linalg.norm(r_vel_n)

        if self.subject_mode == 1: # Robot subject
            subj_p, subj_v, obst_p, obst_v = Point(r_pos_n[0], r_pos_n[1]), Point(r_vel_n[0], r_vel_n[1]), Point(h_pos_n[0], h_pos_n[1]), Point(h_vel_n[0], h_vel_n[1])
        else: # Human subject
            subj_p, subj_v, obst_p, obst_v = Point(h_pos_n[0], h_pos_n[1]), Point(h_vel_n[0], h_vel_n[1]), Point(r_pos_n[0], r_pos_n[1]), Point(r_vel_n[0], r_vel_n[1])

        recalculated_risk, collision = compute_risk(subj_p, obst_p, subj_v, obst_v)

        row = {
            'Timestamp': curr_time,
            'Vr': r_v_noisy,
            'Vh': h_v_noisy,
            'Risk': recalculated_risk,
            'Collision': int(collision)
        }
        self.data_rows.append(row)
        
        if len(self.data_rows) % 100 == 0:
            rospy.loginfo(f"Collected {len(self.data_rows)} samples ({curr_time - self.first_timestamp:.1f}s)...")

    def save_data(self):
        if not self.data_rows:
            rospy.logwarn("No data collected!")
            return
            
        df = pd.DataFrame(self.data_rows)

        # No hand-made lag shifts: the columns are time-aligned and the causal
        # discovery searches the lags itself (they were tuned to the old
        # risk formula, which embedded the subject velocity)

        df.to_csv(self.output_file, index=False)
        rospy.loginfo(f"Saved {len(df)} rows to {self.output_file}")

if __name__ == '__main__':
    rospy.init_node('metrics_extractor')
    extractor = MetricsExtractor()
    rospy.on_shutdown(extractor.save_data)
    rospy.spin()
