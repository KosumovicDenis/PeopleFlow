#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>
#include <map>

#include <pedsim_msgs/TrackedPersons.h>
#include <pedsim_msgs/AgentStates.h>

namespace gazebo
{
    class ActorPosesPlugin : public WorldPlugin {
    public:
        ActorPosesPlugin() : WorldPlugin() {}

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
            this->world_ = _world;
            if (!ros::isInitialized()) {
                ROS_ERROR("ROS not initialized");
                return;
            }
            rosNode.reset(new ros::NodeHandle("gazebo_client"));
            
            // Subscribe to PedSim agents
            ros::SubscribeOptions so = ros::SubscribeOptions::create<pedsim_msgs::AgentStates>(
                "/pedsim_simulator/simulated_agents", 
                10, // Increased queue size to avoid dropping messages
                boost::bind(&ActorPosesPlugin::OnRosMsg, this, _1), 
                ros::VoidPtr(), &rosQueue);
            rosSub = rosNode->subscribe(so);

            rosQueueThread = std::thread(std::bind(&ActorPosesPlugin::QueueThread, this));
            
            ROS_INFO("ActorPosesPlugin loaded and optimized with model caching.");
        }

        // Helper to get or cache model pointer
        physics::ModelPtr GetModelPtr(const std::string& name) {
            auto it = model_cache_.find(name);
            if (it != model_cache_.end()) {
                return it->second;
            }
            
            physics::ModelPtr model = world_->ModelByName(name);
            if (model) {
                model_cache_[name] = model;
            }
            return model;
        }

        void OnRosMsg(const pedsim_msgs::AgentStatesConstPtr msg) {
            for (const auto& agent : msg->agent_states) {
                std::string agent_id_str = std::to_string(agent.id);
                physics::ModelPtr model = GetModelPtr(agent_id_str);

                if (model) {
                    ignition::math::Pose3d gzb_pose;
                    gzb_pose.Pos().Set(agent.pose.position.x + MODEL_OFFSET_X,
                                       agent.pose.position.y + MODEL_OFFSET_Y,
                                       agent.pose.position.z + MODEL_OFFSET);
                    gzb_pose.Rot().Set(agent.pose.orientation.w,
                                       agent.pose.orientation.x,
                                       agent.pose.orientation.y,
                                       agent.pose.orientation.z);

                    try {
                        // SetWorldPose is heavy, but without the double loop it's manageable
                        model->SetWorldPose(gzb_pose);
                    }
                    catch (gazebo::common::Exception gz_ex) {
                        ROS_ERROR("Error setting pose for agent %s: %s", agent_id_str.c_str(), gz_ex.GetErrorStr().c_str());
                    }
                }
            }
        }

    private: 
        void QueueThread() {
            static const double timeout = 0.01; // Faster polling
            while (rosNode->ok()) {
                rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Subscriber rosSub;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        physics::WorldPtr world_;
        
        // Cache to store model pointers and avoid O(N^2) searches
        std::map<std::string, physics::ModelPtr> model_cache_;

        const float MODEL_OFFSET = 0.0;
        const float MODEL_OFFSET_X = 0.0;
        const float MODEL_OFFSET_Y = 0.0;
    };

    GZ_REGISTER_WORLD_PLUGIN(ActorPosesPlugin)
}
