#!/usr/bin/env python

import rospy
import pickle
import traceback
from pedsim_srvs.srv import GetNextDestination, GetNextDestinationResponse
from hrisim_util.Agent import Agent
import hrisim_util.ros_utils as ros_utils
from robot_srvs.srv import VisualisePath

class PedsimBridge():
    def __init__(self):
        # Service to handle destination requests from agents
        rospy.Service('get_next_destination', GetNextDestination, self.handle_get_next_destination)
        rospy.loginfo('Simplified ROS service /get_next_destination advertised')
        
        # Initialize causal simulation parameters
        rospy.set_param("/peopleflow/trigger_B", False)
        rospy.loginfo("Causal trigger /peopleflow/trigger_B initialized to False")

    def load_agents(self, req):
        """Load agents from the ROS parameter server."""
        agent_id = str(req.agent_id)
        agents_param = rospy.get_param(f'/peopleflow/agents/{agent_id}', None)
        
        # We no longer use SCHEDULE, so we pass None
        if agents_param is not None:
            a = Agent.from_dict(agents_param, None, G)
        else:
            a = Agent(agent_id, None, G)
        
        a.x = req.origin.x
        a.y = req.origin.y
        a.isStuck = req.is_stuck
        return a

    def save_agents(self, agent):
        """Save agents to the ROS parameter server."""
        rospy.set_param(f'/peopleflow/agents/{agent.id}', agent.to_dict())

    def handle_get_next_destination(self, req):
        """
        Specialized navigation logic: 
        Agent A (id=1) loops WP_A1 <-> WP_A2. Reaching WP_A2 triggers Agent B.
        Agent B (id=2) waits at WP_B1 until triggered, then does WP_B1 -> WP_B2 -> WP_B1 and stops.
        """
        try:
            # Load agent state
            agent = self.load_agents(req)
            agent_id = str(req.agent_id)
            task_duration = 0 # Default: continuous movement

            # Check if agent has reached destination or is stuck
            if agent.isFree or agent.isStuck:
                if agent_id == "1": # Agent A (Master)
                    # Loop logic for A
                    if agent.pastFinalDest == "WP_A1":
                        next_destination = "WP_A2"
                    else:
                        next_destination = "WP_A1"
                    
                    agent.setTask(next_destination, duration=0, isStuck=agent.isStuck)
                    rospy.loginfo(f"Agent A (1) moving to {next_destination}")
                    
                    # TRIGGER logic: if A just reached WP_A2
                    if agent.pastFinalDest == "WP_A2":
                        rospy.set_param("/peopleflow/trigger_B", True)
                        rospy.logwarn(">>> AGENT A REACHED WP_A2 - TRIGGERING B <<<")

                elif agent_id == "2": # Agent B (Slave)
                    trigger = rospy.get_param("/peopleflow/trigger_B", False)
                    
                    if trigger:
                        if agent.pastFinalDest == "WP_B1":
                            next_destination = "WP_B2"
                            rospy.loginfo("Agent B (2) triggered - Moving to WP_B2")
                        elif agent.pastFinalDest == "WP_B2":
                            # End of cycle: return to WP_B1 and reset trigger
                            next_destination = "WP_B1"
                            rospy.set_param("/peopleflow/trigger_B", False)
                            rospy.logwarn(">>> AGENT B FINISHED CYCLE - RESETTING TRIGGER <<<")
                        else:
                            # Initial or other state: ensure at B1
                            next_destination = "WP_B1"
                    else:
                        # Idle at WP_B1
                        next_destination = "WP_B1"
                        task_duration = 1.0 # Wait 1s between checks when Idle
                    
                    agent.setTask(next_destination, duration=0, isStuck=agent.isStuck)
                
                else:
                    # Default random behavior for other agents (including robot if needed)
                    next_destination = agent.selectDestination(None, list(req.destinations))
                    agent.setTask(next_destination, duration=0, isStuck=agent.isStuck)

            # Prepare response from agent's internal path
            wpname, wp = agent.nextWP         
            response = GetNextDestinationResponse(
                destination_id=wpname, 
                destination=wp, 
                destination_radius=WPS[wpname]["r"] if wpname in WPS else 1.0,
                task_duration=task_duration
            )
            
            # Save updated agent state
            self.save_agents(agent)
            return response

        except Exception as e:
            rospy.logerr(f"Agent {req.agent_id} generated error: {str(e)}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
            return None

if __name__ == '__main__':
    rospy.init_node('peopleflow_pedsim_bridge')

    # Load parameters
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    g_path = str(rospy.get_param("~g_path"))
    
    # Load graph and publish it
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        ros_utils.load_graph_to_rosparam(G, "/peopleflow/G")

        # Optional: visualise path on startup
        try:
            rospy.wait_for_service('/graph/path/show', timeout=2.0)
            graph_path_show = rospy.ServiceProxy('/graph/path/show', VisualisePath)
            graph_path_show("")
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logwarn("Visualization service /graph/path/show not available")

    # Initialize bridge
    pedsimBridge = PedsimBridge()
    rospy.loginfo("Pedsim Bridge started (Continuous Navigation Mode)!")

    rospy.spin()
