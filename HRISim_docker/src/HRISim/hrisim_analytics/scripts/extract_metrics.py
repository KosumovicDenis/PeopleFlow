#!/usr/bin/env python

import rospy
import pandas as pd
import os
import message_filters
from pedsim_msgs.msg import AgentStates
from hrisim_risk.msg import Risk
import math

class MetricsExtractor:
    def __init__(self):
        # Parameters
        self.bag_name = rospy.get_param('~bag_name', 'data_log')
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/ros_ws/src/HRISim/hrisim_analytics/data'))
        
        # Adjustable IDs via parameters
        self.robot_id = int(rospy.get_param('~robot_id', 1))
        self.human_id = int(rospy.get_param('~human_id', 0))
        
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        self.output_file = os.path.join(self.output_dir, f"{self.bag_name}.csv")
        self.data_rows = []

        # Subscribers
        sub_agents = message_filters.Subscriber("/pedsim_simulator/simulated_agents", AgentStates)
        sub_risk = message_filters.Subscriber("/hri/risk", Risk)

        # Synchronizer
        self.ats = message_filters.ApproximateTimeSynchronizer(
            [sub_agents, sub_risk], 
            queue_size=100, 
            slop=0.1,
            allow_headerless=True
        )
        self.ats.registerCallback(self.callback)
        
        rospy.loginfo(f"Metrics Extractor initialized. Robot ID: {self.robot_id}, Human ID: {self.human_id}")
        rospy.loginfo(f"Saving to: {self.output_file}")

    def callback(self, agents_msg, risk_msg):
        h_v = 0.0
        r_v = 0.0
        found_h = False
        found_r = False

        for agent in agents_msg.agent_states:
            if agent.id == self.human_id:
                h_v = math.sqrt(agent.twist.linear.x**2 + agent.twist.linear.y**2)
                found_h = True
            elif agent.id == self.robot_id:
                r_v = math.sqrt(agent.twist.linear.x**2 + agent.twist.linear.y**2)
                found_r = True
        
        if found_h and found_r:
            row = {
                'timestamp': agents_msg.header.stamp.to_sec(),
                'robot_velocity': r_v,
                'human_velocity': h_v,
                'risk': risk_msg.risk.data
            }
            self.data_rows.append(row)
            
            if len(self.data_rows) % 100 == 0:
                rospy.loginfo(f"Collected {len(self.data_rows)} samples...")

    def save_data(self):
        if not self.data_rows:
            rospy.logwarn(f"No data collected! Are Robot ID ({self.robot_id}) and Human ID ({self.human_id}) correct?")
            return
            
        df = pd.DataFrame(self.data_rows)
        df.to_csv(self.output_file, index=False)
        rospy.loginfo(f"Successfully saved {len(self.data_rows)} rows to {self.output_file}")

if __name__ == '__main__':
    rospy.init_node('metrics_extractor')
    extractor = MetricsExtractor()
    rospy.on_shutdown(extractor.save_data)
    rospy.spin()
