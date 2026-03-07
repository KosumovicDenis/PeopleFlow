#!/usr/bin/env python

import rospy
import pickle
import traceback
import xml.etree.ElementTree as ET
import rospkg
from pedsim_srvs.srv import GetNextDestination, GetNextDestinationResponse
from hrisim_util.Agent import Agent
import hrisim_util.ros_utils as ros_utils
from robot_srvs.srv import VisualisePath

class PedsimBridge():
    def __init__(self, scenario_name):
        # Data-driven dependencies
        self.dependencies = {}     
        self.active_triggers = {} 
        self.load_dependencies(scenario_name)
        
        # CLEAR PERSISTED STATE
        if rospy.has_param("/peopleflow/agents"):
            rospy.delete_param("/peopleflow/agents")
            rospy.loginfo("Cleaned up persisted agent states from parameter server")
        
        # Session management: keep track of agents seen in THIS session
        self.seen_agents = set()

        # Service to handle destination requests from agents
        # Advertised at the end to ensure initialization is complete
        rospy.Service('get_next_destination', GetNextDestination, self.handle_get_next_destination)
        rospy.loginfo('Simplified ROS service /get_next_destination advertised')

    def load_dependencies(self, scenario_name):
        try:
            rospack = rospkg.RosPack()
            scenario_path = rospack.get_path('pedsim_simulator') + "/scenarios/" + scenario_name + ".xml"
            
            tree = ET.parse(scenario_path)
            root = tree.getroot()
            
            deps_node = root.find('dependencies')
            if deps_node is not None:
                for dep in deps_node.findall('dependency'):
                    waiter_id = dep.get('waiter_id')
                    trigger_id = dep.get('trigger_id')
                    trigger_wp = dep.get('trigger_wp')
                    
                    self.dependencies[waiter_id] = {
                        'trigger_id': trigger_id,
                        'trigger_wp': trigger_wp
                    }
                    self.active_triggers[(trigger_id, trigger_wp)] = False
                    rospy.loginfo(f"Dependency loaded: Agent {waiter_id} waits for Agent {trigger_id} @ {trigger_wp}")
        except Exception as e:
            rospy.logerr(f"Failed to load dependencies: {str(e)}")

    def load_agents(self, req):
        """Load agents from the ROS parameter server."""
        agent_id = str(req.agent_id)
        agents_param = rospy.get_param(f'/peopleflow/agents/{agent_id}', None)
        
        if agents_param is not None:
            a = Agent.from_dict(agents_param, None, G)
        else:
            a = Agent(agent_id, None, G)
        
        # SESSION RESET: If this is the first time we see this agent in this session,
        # wipe its memory of previous waypoints to avoid ghost triggers.
        if agent_id not in self.seen_agents:
            a.pastFinalDest = None
            self.seen_agents.add(agent_id)
            rospy.loginfo(f"Agent {agent_id} session initialized")

        a.x = req.origin.x
        a.y = req.origin.y
        a.isStuck = req.is_stuck
        return a

    def save_agents(self, agent):
        """Save agents to the ROS parameter server."""
        rospy.set_param(f'/peopleflow/agents/{agent.id}', agent.to_dict())

    def handle_get_next_destination(self, req):
        """
        Generic data-driven navigation logic with session-aware reset.
        """
        try:
            # Load agent state
            agent = self.load_agents(req)
            agent_id = str(req.agent_id)
            task_duration = 0 
            potential_dests = list(req.destinations)

            if agent.isFree or agent.isStuck:
                if not potential_dests:
                    return GetNextDestinationResponse(destination_id="none", destination=req.origin, destination_radius=1.0, task_duration=1.0)
                # 1. WAITER LOGIC: Check if we can move or must stay idle
                if agent_id in self.dependencies:
                    dep = self.dependencies[agent_id]
                    trigger_key = (dep['trigger_id'], dep['trigger_wp'])
                    is_triggered = self.active_triggers.get(trigger_key, False)
                    
                    if is_triggered:
                        # Replicating cycle logic
                        if agent.pastFinalDest == potential_dests[0]:
                            next_destination = potential_dests[1]
                        elif agent.pastFinalDest == potential_dests[1]:
                            next_destination = potential_dests[0]
                            # Reset trigger after completing the cycle
                            self.active_triggers[trigger_key] = False
                            rospy.logwarn(f">>> AGENT {agent_id} COMPLETED CYCLE - TRIGGER RESET <<<")
                        else:
                            next_destination = potential_dests[0]
                    else:
                        # NOT TRIGGERED: Stay at idle position
                        next_destination = potential_dests[0]
                        task_duration = 1.0 
                
                else:
                    # 2. NORMAL LOGIC: Continuous Cycle
                    if agent.pastFinalDest in potential_dests:
                        idx = potential_dests.index(agent.pastFinalDest)
                        next_destination = potential_dests[(idx + 1) % len(potential_dests)]
                    else:
                        next_destination = potential_dests[0]

                # Apply the task (updates pastFinalDest to the WP just reached)
                agent.setTask(next_destination, duration=0, isStuck=agent.isStuck)
                
                # 3. TRIGGER LOGIC: Set trigger if master reaches target WP
                # Check for None to avoid start-up trigger from session reset
                if agent.pastFinalDest is not None:
                    for (t_id, t_wp) in self.active_triggers.keys():
                        if t_id == agent_id and agent.pastFinalDest == t_wp:
                            if not self.active_triggers[(t_id, t_wp)]: # Avoid multiple logs for same arrival
                                self.active_triggers[(t_id, t_wp)] = True
                                rospy.logwarn(f">>> TRIGGER ACTIVATED: Agent {agent_id} reached {t_wp} <<<")

            # Prepare response
            wpname, wp = agent.nextWP         
            response = GetNextDestinationResponse(
                destination_id=wpname, 
                destination=wp, 
                destination_radius=WPS[wpname]["r"] if wpname in WPS else 1.0,
                task_duration=task_duration
            )
            self.save_agents(agent)
            return response

        except Exception as e:
            rospy.logerr(f"Agent {req.agent_id} generated error: {str(e)}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
            return None

if __name__ == '__main__':
    rospy.init_node('peopleflow_pedsim_bridge')

    # Load parameters
    scenario_name = str(rospy.get_param("~scenario", "warehouse"))
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
    pedsimBridge = PedsimBridge(scenario_name)
    rospy.loginfo(f"Pedsim Bridge started for scenario '{scenario_name}'!")

    rospy.spin()
