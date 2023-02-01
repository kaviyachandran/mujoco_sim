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

#include "mj_hw_interface.h"

MjHWInterface::MjHWInterface(const std::string &robot)
{
    joint_names = MjSim::joint_names[robot];
    std::size_t num_joints = joint_names.size();
    joint_positions.resize(num_joints, 0.);
    joint_velocities.resize(num_joints, 0.);
    joint_efforts.resize(num_joints, 0.);

    joint_positions_command.resize(mj_sim.position_controlled_joints.size(), 0.);
    // joint_velocities_command.resize(num_joints, 0.);
    joint_efforts_command.resize(num_joints, 0.);

    for (std::size_t i = 0; i < num_joints; i++)
    {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &joint_positions[i], &joint_velocities[i], &joint_efforts[i]);
        joint_state_interface.registerHandle(joint_state_handle);

        if(std::find(mj_sim.position_controlled_joints.begin(),mj_sim.position_controlled_joints.end(), joint_names[i]) != mj_sim.position_controlled_joints.end())
        {
            hardware_interface::JointHandle joint_handle_position(joint_state_interface.getHandle(joint_names[i]), &joint_positions_command[i]);
            position_joint_interface.registerHandle(joint_handle_position);    
        }
        else{
            hardware_interface::JointHandle joint_handle_effort(joint_state_interface.getHandle(joint_names[i]), &joint_efforts_command[i]);
            effort_joint_interface.registerHandle(joint_handle_effort);
        }
        

        // hardware_interface::JointHandle joint_handle_velocity(joint_state_interface.getHandle(joint_names[i]), &joint_velocities_command[i]);
        // velocity_joint_interface.registerHandle(joint_handle_velocity);

    }
    registerInterface(&joint_state_interface);
    registerInterface(&position_joint_interface);
    // registerInterface(&velocity_joint_interface);
    registerInterface(&effort_joint_interface);
}

MjHWInterface::~MjHWInterface()
{
}

void MjHWInterface::read()
{
    mj_inverse(m, d);
    for (std::size_t i = 0; i < joint_names.size(); i++)
    {
        const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_names[i].c_str());
        const int qpos_id = m->jnt_qposadr[joint_id];
        const int dof_id = m->jnt_dofadr[joint_id];
        joint_positions[i] = d->qpos[qpos_id];
        joint_velocities[i] = d->qvel[dof_id];
        joint_efforts[i] = d->qfrc_inverse[dof_id];
    }

    const int sensorId = mj_name2id(m, mjtObj::mjOBJ_JOINT, "force_ll");
    int adr = m->sensor_adr[sensorId];
    int dim = m->sensor_dim[sensorId];
    mjtNum sensor_data[dim];
    mju_copy(sensor_data, &d->sensordata[adr], dim);
    std::cout << "sensor data " << d->sensordata[0] << " " << d->sensordata[1] << " "
    << d->sensordata[2] << " " << d->sensordata[3] << " " << d->sensordata[4] << " "
    << d->sensordata[5] << std::endl;
}

void MjHWInterface::write()
{
    for (std::size_t i = 0; i < joint_names.size(); i++)
    {
        if (std::find(MjSim::joint_ignores.begin(), MjSim::joint_ignores.end(), joint_names[i]) == MjSim::joint_ignores.end()
        && std::find(mj_sim.position_controlled_joints.begin(), mj_sim.position_controlled_joints.end(), joint_names[i]) == 
            mj_sim.position_controlled_joints.end())
        {
            const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_names[i].c_str());
            const int dof_id = m->jnt_dofadr[joint_id];
            MjSim::u[dof_id] = joint_efforts_command[i];
        }
    }

    for (std::size_t j=0; j < mj_sim.position_controlled_joints.size(); j++)
    {
        const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, mj_sim.position_controlled_joints[j].c_str());
        const int dof_id = m->jnt_dofadr[joint_id];
        d->qpos[dof_id] = joint_positions_command[j];
        d->qvel[dof_id] = 0;
        d->qfrc_applied[dof_id] = 0;
    }
}

void MjHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                             const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    for (const hardware_interface::ControllerInfo &stop_controller : stop_list)
    {
        for (const hardware_interface::InterfaceResources &interface_resource : stop_controller.claimed_resources)
        {
            for (const std::string &joint_name : interface_resource.resources)
            {
                if (std::find(MjSim::joint_ignores.begin(), MjSim::joint_ignores.end(), joint_name) == MjSim::joint_ignores.end())
                {

                    std::vector<std::string>::iterator it = std::find(joint_names.begin(), joint_names.end(), joint_name);
                    if (it != joint_names.end())
                    {
                        joint_efforts_command[it - joint_names.begin()] = 0.;
                    }
                }
            }
        }
    }
}