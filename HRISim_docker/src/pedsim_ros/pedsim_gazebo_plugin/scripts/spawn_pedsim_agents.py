#!/usr/bin/env python
"""
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
from rospkg import RosPack
from pedsim_msgs.msg  import AgentStates
import os

# Global dictionary to cache model XMLs
model_cache = {}

def get_model_xml(model_name):
    if model_name in model_cache:
        return model_cache[model_name]
    
    rospack = RosPack()
    pkg_path = rospack.get_path('pedsim_gazebo_plugin')
    
    # Map requested model name to actual model directory
    # Default is modello_umano if not specified or unknown
    target_model = model_name if model_name in ["modello_robot", "modello_umano"] else "modello_umano"
    
    model_path = os.path.join(pkg_path, "models", target_model, "model.sdf")
    
    # If the specific model file doesn't exist, fallback to default actor_model.sdf
    if not os.path.exists(model_path):
        rospy.logwarn("Model path %s not found, falling back to default actor_model.sdf", model_path)
        model_path = os.path.join(pkg_path, "models", "actor_model.sdf")

    try:
        with open(model_path, 'r') as f:
            xml_string = f.read()
            model_cache[model_name] = xml_string
            return xml_string
    except Exception as e:
        rospy.logerr("Error reading model file %s: %s", model_path, str(e))
        return None

def cb_actor_poses(actors):
    global AGENT_SPAWNED
    if not AGENT_SPAWNED:
        for actor in actors.agent_states:
            actor_id = str( actor.id )
            actor_pose = actor.pose
            
            # Choose model dynamically
            model_to_use = actor.model if actor.model else "modello_umano"
            rospy.loginfo("Spawning model: actor_id = %s, model = %s", actor_id, model_to_use)
            
            xml_string = get_model_xml(model_to_use)
            if not xml_string:
                continue

            model_pose = Pose(Point(x= actor_pose.position.x,
                                    y= actor_pose.position.y,
                                    z= actor_pose.position.z),
                            Quaternion(actor_pose.orientation.x,
                                        actor_pose.orientation.y,
                                        actor_pose.orientation.z,
                                        actor_pose.orientation.w) )

            try:
                spawn_model(actor_id, xml_string, "", model_pose, "world")
            except Exception as e:
                rospy.logerr("Error spawning model %s: %s", actor_id, str(e))
        rospy.logwarn("All autonomous agents have been spawn")
        AGENT_SPAWNED = True
        
def cb_teleop_actor_poses(actors):
    global TELEOP_AGENT_SPAWNED
    if not TELEOP_AGENT_SPAWNED:
        for actor in actors.agent_states:
            actor_id = str( actor.id )
            actor_pose = actor.pose
            
            model_to_use = actor.model if actor.model else "modello_umano"
            rospy.loginfo("Spawning model: actor_id = %s, model = %s", actor_id, model_to_use)
            
            xml_string = get_model_xml(model_to_use)
            if not xml_string:
                continue

            model_pose = Pose(Point(x= actor_pose.position.x,
                                y= actor_pose.position.y,
                                z= actor_pose.position.z),
                            Quaternion(actor_pose.orientation.x,
                                        actor_pose.orientation.y,
                                        actor_pose.orientation.z,
                                        actor_pose.orientation.w) )

            try:
                spawn_model(actor_id, xml_string, "", model_pose, "world")
            except Exception as e:
                rospy.logerr("Error spawning model %s: %s", actor_id, str(e))
        rospy.logwarn("All teleop agents have been spawn")
        TELEOP_AGENT_SPAWNED = True

if __name__ == '__main__':

    rospy.init_node("spawn_pedsim_agents")
    rate = rospy.Rate(10)
    
    global AGENT_SPAWNED, TELEOP_AGENT_SPAWNED
    AGENT_SPAWNED = not bool(rospy.get_param('/pedsim_simulator/spawn_agent', True))
    TELEOP_AGENT_SPAWNED = not bool(rospy.get_param('/pedsim_simulator/spawn_teleop_agent', False))
    TIMEOUT = float(rospy.get_param('/pedsim_simulator/spawn_timeout', 10))

    try:
        print("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/spawn_sdf_model", timeout=5.0)
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        print("service: spawn_sdf_model is available ....")
        rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, cb_actor_poses)
        rospy.Subscriber("/ped/control/gz_persons", AgentStates, cb_teleop_actor_poses)

        init = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - init >= TIMEOUT: 
                rospy.loginfo("Spawn timeout reached")
                break
            if AGENT_SPAWNED and TELEOP_AGENT_SPAWNED:
                rospy.loginfo("All agents spawned successfully")
                break
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Spawn script encountered an error: %s", str(e))
    finally:
        rospy.loginfo("Shutting down spawn_pedsim_agents node")
