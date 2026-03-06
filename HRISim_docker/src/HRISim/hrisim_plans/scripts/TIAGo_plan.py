import math
import os
import pickle
import random
import sys
import rospy
import traceback

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *
from robot_msgs.msg import BatteryStatus
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction
import actionlib
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants
import networkx as nx
from robot_srvs.srv import NewTask, FinishTask, VisualisePath
from nav_msgs.msg import Odometry

def send_goal(p, next_dest, nextnext_dest=None):
    pos = nx.get_node_attributes(G, 'pos')
    x, y = pos[next_dest]
    if nextnext_dest is not None:
        x2, y2 = pos[nextnext_dest]
        angle = math.atan2(y2-y, x2-x)
        inputs = [x, y, angle, TIME_THRESHOLD]
    else:
        inputs = [x, y, 0, TIME_THRESHOLD]
    p.exec_action('goto', "_".join([str(input) for input in inputs]))
    
def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def get_next_goal():
    global ROBOT_CLOSEST_WP
    # Simplified logic: pick a random waypoint from the graph that is not the current one
    potential_goals = [node for node in G.nodes if node != ROBOT_CLOSEST_WP]
    if not potential_goals:
        potential_goals = list(G.nodes)
    
    if potential_goals:
        selected = random.choice(potential_goals)
        return selected, constants.Task.DELIVERY, True
    return None, None, False

def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
        
    global NEXT_GOAL, QUEUE
    
    ros_utils.wait_for_service('/hrisim/new_task')
    ros_utils.wait_for_service('/hrisim/finish_task')
    graph_path_show = rospy.ServiceProxy('/graph/path/show', VisualisePath)
    new_task_service = rospy.ServiceProxy('/hrisim/new_task', NewTask)
    finish_task_service = rospy.ServiceProxy('/hrisim/finish_task', FinishTask)
    
    rospy.set_param('/hrisim/robot_busy', False)
    PLAN_ON = True
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)
    
    while PLAN_ON:
        try:
            if not rospy.get_param('/hrisim/robot_busy') and len(QUEUE) == 0:
                NEXT_GOAL, TASK, PLAN_ON = get_next_goal()
                if NEXT_GOAL is None: 
                    rospy.sleep(1.0)
                    continue
                
                # Find path in the graph
                try:
                    QUEUE = nx.astar_path(G, ROBOT_CLOSEST_WP, NEXT_GOAL, heuristic=heuristic, weight='weight')
                    rospy.logwarn(f"New robot path: {QUEUE}")
                    graph_path_show(','.join(QUEUE))
                    task_id = new_task_service(NEXT_GOAL, QUEUE).task_id
                except nx.NetworkXNoPath:
                    rospy.logerr(f"No path from {ROBOT_CLOSEST_WP} to {NEXT_GOAL}")
                    QUEUE = []
                    rospy.sleep(1.0)
                    continue

            if not rospy.get_param('/hrisim/robot_busy') and len(QUEUE) > 0:
                next_sub_goal = QUEUE.pop(0)
                rospy.logwarn(f"Robot moving to: {next_sub_goal}")
                nextnext_sub_goal = QUEUE[0] if len(QUEUE) > 0 else None
                
                send_goal(p, next_sub_goal, nextnext_sub_goal)
                
                if len(QUEUE) == 0:
                    finish_task_service(task_id, constants.TaskResult.SUCCESS.value)
                    
            rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"Error in Planning loop: {e}")
            rospy.logerr(traceback.format_exc())
            rospy.sleep(1.0)

def cb_robot_closest_wp(wp: String):
    global ROBOT_CLOSEST_WP
    ROBOT_CLOSEST_WP = wp.data

if __name__ == "__main__":  
    ROBOT_CLOSEST_WP = None
    NEXT_GOAL = None
    QUEUE = []
    
    p = PNPCmd()
    
    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        # REMOVED: G.remove_node("parking") as requested
        
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)
    TIME_THRESHOLD = ros_utils.wait_for_param("/hrisim/abort_time_threshold")

    # Wait for first WP position
    while ROBOT_CLOSEST_WP is None and not rospy.is_shutdown():
        rospy.loginfo("Waiting for robot position...")
        rospy.sleep(0.5)

    p.begin()
    Plan(p)
    p.end()
