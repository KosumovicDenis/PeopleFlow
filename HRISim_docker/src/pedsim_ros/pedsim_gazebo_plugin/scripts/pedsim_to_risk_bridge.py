#!/usr/bin/env python3

import rospy
import tf_conversions
import traceback
from pedsim_msgs.msg import AgentStates
from peopleflow_msgs.msg import RobotState, HumanState
from geometry_msgs.msg import Pose2D, Twist, Point

class PedsimToRiskBridge:
    def __init__(self):
        rospy.init_node('pedsim_to_risk_bridge')
        
        # Publishers
        self.pub_robot = rospy.Publisher('/roscausal/robot', RobotState, queue_size=10)
        self.pub_human = rospy.Publisher('/roscausal/human', HumanState, queue_size=10)
        
        # Subscriber
        self.sub_pedsim = rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, self.cb_pedsim)
        
        rospy.loginfo("Pedsim-to-Risk Bridge started and listening to /pedsim_simulator/simulated_agents")

    def cb_pedsim(self, msg):
        try:
            found_robot = False
            found_human = False
            
            for agent in msg.agent_states:
                # ID 1: Robot
                if agent.id == 1:
                    out_msg = RobotState()
                    self.fill_msg(agent, out_msg, msg.header.stamp)
                    self.pub_robot.publish(out_msg)
                    found_robot = True
                
                # ID 0: Human
                elif agent.id == 0:
                    out_msg = HumanState()
                    self.fill_msg(agent, out_msg, msg.header.stamp)
                    self.pub_human.publish(out_msg)
                    found_human = True
            
            if not found_robot and not found_human:
                rospy.logwarn_throttle(10, "Bridge receiving messages but no agent with ID 0 or 1 found.")
                
        except Exception as e:
            rospy.logerr(f"Error in bridge callback: {e}")
            rospy.logerr(traceback.format_exc())

    def fill_msg(self, agent, out_msg, stamp):
        # Header
        out_msg.header.stamp = stamp
        out_msg.header.frame_id = "world"
        
        # Pose2D
        out_msg.pose2D.x = agent.pose.position.x
        out_msg.pose2D.y = agent.pose.position.y
        
        q = agent.pose.orientation
        euler = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        out_msg.pose2D.theta = euler[2]
        
        # Twist (Explicitly copy fields)
        out_msg.twist.linear.x = agent.twist.linear.x
        out_msg.twist.linear.y = agent.twist.linear.y
        out_msg.twist.linear.z = agent.twist.linear.z
        out_msg.twist.angular.x = agent.twist.angular.x
        out_msg.twist.angular.y = agent.twist.angular.y
        out_msg.twist.angular.z = agent.twist.angular.z
        
        # Goal
        out_msg.goal.x = agent.goal.x
        out_msg.goal.y = agent.goal.y
        out_msg.goal.z = agent.goal.z
        
        return out_msg

if __name__ == '__main__':
    try:
        bridge = PedsimToRiskBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal bridge error: {e}")
