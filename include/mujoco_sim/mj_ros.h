// Copyright (c) 2022, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "mj_sim.h"
#include "mujoco_msgs/DestroyObject.h"
#include "mujoco_msgs/ObjectInfo.h"
#include "mujoco_msgs/ObjectState.h"
#include "mujoco_msgs/ObjectStateArray.h"
#include "mujoco_msgs/ObjectStatus.h"
#include "mujoco_msgs/SpawnObject.h"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

/**
 * @brief Set tmp_model_name, world_path and odom_joints
 */
void set_params();

class CmdVelCallback
{

public:
    CmdVelCallback() {}

    CmdVelCallback(const size_t in_id, const std::string &in_robot);

private:
    size_t id = 0;
    std::string robot;

public:
    void callback(const geometry_msgs::Twist &msg);
};

enum EObjectType : std::int8_t
{
    None = 0,
    Robot = 1,
    World = 2,
    SpawnedObject = 3
};

class MjRos
{
public:
    MjRos() = default;

    ~MjRos();

public:
    /**
     * @brief Initialize publishers, subscribers and MjRos members from rosparam
     *
     */
    void init();

    /**
     * @brief Publish tf
     *
     */
    void publish_tf(const EObjectType object_type = EObjectType::None);

    /**
     * @brief Publish markers
     *
     */
    void publish_marker_array(const EObjectType object_type = EObjectType::None);

    /**
     * @brief Publish object states
     *
     */
    void publish_object_state_array(const EObjectType object_type = EObjectType::None);

    /**
     * @brief Publish world joint states
     *
     */
    void publish_joint_states(const EObjectType object_type = EObjectType::None);

    /**
     * @brief Publish base pose
     *
     */
    void publish_base_pose();

    /**
     * @brief Publish sensor data
     *
     */
    void publish_sensor_data();

    /**
     * @brief Run spawn and destroy objects
     *
     */
    void spawn_and_destroy_objects();

private:
    /**
     * @brief Reset the robot to the initial state service
     *
     * @param req Request
     * @param res Response
     * @return true Success
     * @return false Fail
     */
    bool reset_robot_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

    /**
     * @brief Spawn objects from ROS
     *
     * @param req New object parameters
     * @param res Success or not
     * @return true Success
     * @return false Fail
     */
    bool spawn_objects_service(mujoco_msgs::SpawnObjectRequest &req, mujoco_msgs::SpawnObjectResponse &res);

    /**
     * @brief Spawn objects
     *
     * @param objects Objects to spawn
     */
    void spawn_objects(const std::vector<mujoco_msgs::ObjectStatus> objects);

    /**
     * @brief Destroy objects from ROS
     *
     * @param req Array of object names
     * @param res Success or not
     * @return true Success
     * @return false Fail
     */
    bool destroy_objects_service(mujoco_msgs::DestroyObjectRequest &req, mujoco_msgs::DestroyObjectResponse &res);

    /**
     * @brief Destroy objects
     *
     * @param object_names Object names to destroy
     */
    void destroy_objects(const std::vector<std::string> object_names);

    /**
     * @brief Add marker of an object
     *
     * @param body_id Body id of the object
     */
    void add_marker(const int body_id, const EObjectType object_type);

    /**
     * @brief Set pose of the base
     *
     * @param body_id Body id of the base
     *
     * @param robot_id Robot id
     */
    void set_base_pose(const int body_id, const int robot_id = 0);

    /**
     * @brief Add joint state of the world
     *
     * @param body_id Body id of the body
     */
    void add_joint_states(const int body_id, const EObjectType object_type);

    /**
     * @brief Add state of an object
     *
     * @param body_id Body id of the object
     */
    void add_object_state(const int body_id, const EObjectType object_type);

    /**
     * @brief Set transform of an object
     *
     * @param body_id Transform to set
     * @param body_id Body id of the object
     * @param object_name Name of the object
     * @param root_id Body id of the root
     */
    void set_transform(geometry_msgs::TransformStamped &transform, const int body_id, const std::string &object_name);

    /**
     * @brief Reset the robot to the initial state
     *
     */
    void reset_robot();

public:
    /**
     * @brief Start time of ROS
     *
     */
    static ros::Time ros_start;

private:
    ros::NodeHandle n;

    std::string root_frame_id;

    std::vector<CmdVelCallback *> cmd_vel_callbacks;

    std::vector<ros::Subscriber> cmd_vel_subs;

    ros::ServiceServer reset_robot_server;

    ros::ServiceServer spawn_objects_server;

    ros::ServiceServer destroy_objects_server;

    std::vector<ros::Publisher> base_pose_pubs;

    ros::Publisher marker_array_pub;

    ros::Publisher object_state_array_pub;

    std::map<EObjectType, ros::Publisher> joint_states_pub;

    ros::Publisher sensors_pub;

    tf2_ros::TransformBroadcaster br;

    tf2_ros::StaticTransformBroadcaster static_br;

    std::map<std::string, float> joint_inits;
};