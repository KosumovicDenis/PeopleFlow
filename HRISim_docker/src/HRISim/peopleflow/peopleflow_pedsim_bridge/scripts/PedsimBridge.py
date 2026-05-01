#!/usr/bin/env python

import rospy
import pickle
import traceback
from pedsim_srvs.srv import GetNextDestination, GetNextDestinationResponse
from hrisim_util.Agent import Agent
import hrisim_util.ros_utils as ros_utils
from robot_srvs.srv import VisualisePath
from hrisim_risk.msg import Risk
from geometry_msgs.msg import Point

class PedsimBridge():
    def __init__(self):
        # State for the subject agent
        self.subject_id = str(rospy.get_param("/risk/subject", "0"))
        self.subject_state = "WAITING_AT_CENTER"
        self.collision = False
        
        # Risk subscriber
        rospy.Subscriber('/hri/risk', Risk, self.cb_risk)
        
        # Service to handle destination requests from agents
        rospy.Service('get_next_destination', GetNextDestination, self.handle_get_next_destination)
        rospy.loginfo('Simplified ROS service /get_next_destination advertised')

    def cb_risk(self, msg):
        """Callback for risk status."""
        self.collision = msg.collision.data

    def load_agents(self, req):
        """Load agents from the ROS parameter server."""
        agent_id = str(req.agent_id)
        agents_param = rospy.get_param(f'/peopleflow/agents/{agent_id}', None)
        
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
        Navigation logic with subject blocking at center.
        """
        try:
            agent_id = str(req.agent_id)
            agent = self.load_agents(req)

            # Special logic for the subject agent
            if agent_id == self.subject_id:
                if self.subject_state == "WAITING_AT_CENTER":
                    if self.collision:
                        self.subject_state = "DOING_TOUR"
                        rospy.loginfo(f"Collision detected! Subject {agent_id} starting its tour.")
                    else:
                        # Force destination to WP_CENTER with a short duration to keep polling collision status
                        response = GetNextDestinationResponse(
                            destination_id="WP_CENTER", 
                            destination=Point(x=WPS["WP_CENTER"]["x"], y=WPS["WP_CENTER"]["y"], z=0), 
                            destination_radius=WPS["WP_CENTER"]["r"] if "WP_CENTER" in WPS else 0.5,
                            task_duration=0.1 # Poll every 100ms
                        )
                        return response

            # Normal logic for other agents or subject in DOING_TOUR
            if agent.isFree or agent.isStuck:
                next_destination = agent.selectDestination(None, list(req.destinations))
                
                # Check if subject returned to center to reset state
                if agent_id == self.subject_id and next_destination == "WP_CENTER":
                    self.subject_state = "WAITING_AT_CENTER"
                    rospy.loginfo(f"Subject {agent_id} returned to center. Waiting for next collision.")
                
                agent.setTask(next_destination, duration=0, isStuck=agent.isStuck)
                rospy.logdebug(f"Agent {agent.id} moving to {next_destination}")

            # Prepare response from agent's internal path
            wpname, wp = agent.nextWP         
            response = GetNextDestinationResponse(
                destination_id=wpname, 
                destination=wp, 
                destination_radius=WPS[wpname]["r"] if wpname in WPS else 1.0,
                task_duration=0
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
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    g_path = str(rospy.get_param("~g_path"))
    
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        ros_utils.load_graph_to_rosparam(G, "/peopleflow/G")

        try:
            rospy.wait_for_service('/graph/path/show', timeout=2.0)
            graph_path_show = rospy.ServiceProxy('/graph/path/show', VisualisePath)
            graph_path_show("")
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logwarn("Visualization service /graph/path/show not available")

    pedsimBridge = PedsimBridge()
    rospy.loginfo("Pedsim Bridge started with Collision-Triggered Navigation!")

    rospy.spin()
